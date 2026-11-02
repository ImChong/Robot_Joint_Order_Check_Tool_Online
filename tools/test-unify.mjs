/**
 * Headless checks for the "unify joint order" recommendation and the URDF
 * rewrite. Same setup as test-orderings.mjs: the IIFE scripts run against a
 * stub `window`, no extra packages. Node has no DOMParser, so URDF text is
 * read back with a small parser built on Unify.scanXml — it fills the same
 * fields urdf.js does for the parts the ordering depends on.
 *
 *   node tools/test-unify.mjs
 */
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const window = {};
for (const file of ['assets/js/samples.js', 'assets/js/urdf.js', 'assets/js/orderings.js', 'assets/js/unify.js']) {
  new Function('window', readFileSync(resolve(root, file), 'utf8'))(window);
}

const { URDF, Orderings, Unify, SAMPLES } = window;
const out = [];
let failed = 0;

function eq(actual, expected, msg) {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a === e) {
    out.push('ok   ' + msg);
  } else {
    failed++;
    out.push('FAIL ' + msg);
    out.push('     expected: ' + e);
    out.push('     actual:   ' + a);
  }
}

function throws(fn, msg) {
  try { fn(); } catch (e) { out.push('ok   ' + msg); return; }
  failed++;
  out.push('FAIL ' + msg + ' (did not throw)');
}

/* ── text → model, the fields urdf.js would fill ── */
function parseLite(text) {
  const els = Unify.scanXml(text);
  const robot = els.find((e) => e.parent === -1);
  const kids = (parent, name) => els.filter((e) => e.parent === parent.index && e.name === name);
  const attr = (el, a) => Unify.attrOf(text, el, a);
  const m = {
    format: 'urdf', robotName: attr(robot, 'name') || 'robot',
    links: [], joints: [], jointByName: {},
    ros2ControlJoints: [], ros2ControlBlocks: [],
    warnings: [], errors: [],
    looksLikeXacro: /<\s*xacro:/i.test(text) || /\$\{[^}]*\}/.test(text)
  };
  kids(robot, 'link').forEach((el) => m.links.push({ name: attr(el, 'name'), declared: true }));
  kids(robot, 'joint').forEach((el, i) => {
    const one = (name, a) => { const c = kids(el, name)[0]; return c ? attr(c, a) : null; };
    const type = (attr(el, 'type') || 'revolute').toLowerCase();
    const j = {
      name: attr(el, 'name') || '<unnamed_joint_' + i + '>',
      type, parent: one('parent', 'link') || '', child: one('child', 'link') || '',
      docIndex: i, dof: URDF.dofOf(type), mimic: one('mimic', 'joint')
    };
    m.joints.push(j);
    m.jointByName[j.name] = j;
  });
  kids(robot, 'ros2_control').forEach((block) => {
    const entry = { name: attr(block, 'name') || 'ros2_control', type: attr(block, 'type') || '', joints: [] };
    kids(block, 'joint').forEach((j) => {
      const n = attr(j, 'name');
      if (!n) return;
      entry.joints.push(n);
      if (!m.ros2ControlJoints.includes(n)) m.ros2ControlJoints.push(n);
    });
    m.ros2ControlBlocks.push(entry);
  });
  return m;
}

/* Text with every root-level <joint> and every <ros2_control> joint entry cut
   out — what must survive a rewrite byte for byte. */
function skeleton(text) {
  const els = Unify.scanXml(text);
  const robot = els.find((e) => e.parent === -1);
  const blocks = new Set(els.filter((e) => e.parent === robot.index && e.name === 'ros2_control').map((e) => e.index));
  const cut = els.filter((e) => e.name === 'joint' && (e.parent === robot.index || blocks.has(e.parent)));
  let s = '', at = 0;
  cut.forEach((e) => { s += text.slice(at, e.start) + '<#>'; at = e.end; });
  return s + text.slice(at);
}

function analyse(model) {
  return { model, tree: URDF.buildTree(model) };
}

function pipeline(text) {
  const { model, tree } = analyse(parseLite(text));
  const rec = Unify.recommend(model, tree);
  if (rec.status !== 'found') return { rec };
  const rewritten = Unify.rewriteUrdf(text, model, rec);
  const parsed = parseLite(rewritten);
  return { rec, rewritten, parsed, verified: Unify.verifyRewrite(model, parsed, rec) };
}

const sample = (id) => {
  const s = SAMPLES.find((x) => x.id === id);
  return s.text || readFileSync(resolve(root, 'samples', s.file), 'utf8');
};

/* ── Bundled samples ── */
eq(pipeline(sample('arm')).rec.status, 'already', 'arm sample: already unified, nothing to change');

const quad = pipeline(sample('quadruped')).rec;
eq(quad.status, 'none', 'quadruped sample: no unified order exists');
eq([quad.split.link, quad.split.index, quad.split.dfs, quad.split.bfs, quad.split.forcedBy, quad.split.branches],
  ['trunk', 1, 'FL_thigh_joint', 'FR_hip_joint', 'branches', 4],
  'quadruped sample: four legs keep going below trunk, DFS and BFS split at index 1');
eq([...new Set(quad.after.map((c) => c.id))], ['isaacsim'],
  'quadruped sample: only the breadth-first column is left disagreeing');

const mob = pipeline(sample('ros2control')).rec;
eq(mob.status, 'none', 'mobile manipulator sample: no unified order exists');
eq([mob.split.link, mob.split.forcedBy, mob.split.deep, mob.split.next],
  ['base_link', 'alphabetical', 'arm_shoulder_pan', 'drive_wheel_left'],
  'mobile manipulator: one deep branch, but Gazebo sorts it ahead of the wheels');

const g1 = pipeline(sample('g1_urdf')).rec;
eq(g1.status, 'none', 'Unitree G1 URDF: no unified order exists');
eq(g1.split.forcedBy, 'branches', 'Unitree G1 URDF: legs, waist and arms branch below the pelvis');

const ag = pipeline(sample('arm_gripper'));
eq(ag.rec.status, 'found', 'arm + gripper sample: a unified order exists');
eq(ag.rec.order, [
  'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
  'gripper_mount', 'finger_left_joint', 'finger_right_joint'
], 'arm + gripper: chain first, then the fingers in alphabetical order');
eq([ag.rec.moved, ag.rec.movable, ag.rec.ros2Changed, ag.rec.tagModeOk], [9, 8, 8, true],
  'arm + gripper: 9 joints move, 8 movable, the ros2_control block follows');
eq([...new Set(ag.rec.before.map((c) => c.id))].sort(),
  ['gazebo', 'genesis', 'isaacgym', 'isaacsim', 'mujoco', 'newton', 'pybullet'],
  'arm + gripper: as written, every tree-walking column disagrees with the file');
eq(ag.parsed.joints.map((j) => j.name), ag.rec.order, 'arm + gripper: rewritten file has the joints in the new order');
eq(ag.parsed.ros2ControlBlocks[0].joints,
  ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'finger_left_joint', 'finger_right_joint'],
  'arm + gripper: <ros2_control> entries follow the same order');
eq(skeleton(ag.rewritten), skeleton(sample('arm_gripper')),
  'arm + gripper: everything outside the moved joints is byte-for-byte unchanged');
eq(ag.verified, true, 'arm + gripper: re-parsed rewrite passes verification');
eq(Unify.conflicts(ag.parsed, URDF.buildTree(ag.parsed)), [], 'arm + gripper: no column disagrees after the rewrite');
eq(Unify.recommend(ag.parsed, URDF.buildTree(ag.parsed)).status, 'already',
  'arm + gripper: running the recommendation again reports nothing to do');

/* ── Hand-built cases ── */
function J(name, type, parent, child) {
  return { name, type, parent, child, mimic: null };
}

function modelOf(joints, extra) {
  const m = {
    format: 'urdf', robotName: 't', links: [], joints, jointByName: {},
    ros2ControlJoints: [], ros2ControlBlocks: [], warnings: [], errors: [], looksLikeXacro: false
  };
  const links = [];
  joints.forEach((j, i) => {
    j.docIndex = i;
    m.jointByName[j.name] = j;
    [j.parent, j.child].forEach((l) => { if (l && !links.includes(l)) links.push(l); });
  });
  m.links = links.map((name) => ({ name, declared: true }));
  return Object.assign(m, extra || {});
}

function rec(model) {
  return Unify.recommend(model, URDF.buildTree(model));
}

/* Movable order already agrees; only a fixed joint sits out of tree order —
   PyBullet / Newton count it, so it still needs moving. */
const fixedFirst = rec(modelOf([
  J('tool_fixed', 'fixed', 'l2', 'tool'),
  J('j1', 'revolute', 'base', 'l1'),
  J('j2', 'revolute', 'l1', 'l2')
]));
eq(fixedFirst.status, 'found', 'fixed joint out of tree order: found');
eq(fixedFirst.order, ['j1', 'j2', 'tool_fixed'], 'fixed joint out of tree order: moved to its DFS slot');
eq(fixedFirst.before.every((c) => c.showFixed), true,
  'fixed joint out of tree order: only shows up once fixed joints are counted');

/* Gazebo never sees a fixed-only branch, so where it sits is free: declared
   first it is already fine even though it would sort last ... */
eq(rec(modelOf([
  J('z_camera_mount', 'fixed', 'base', 'camera'),
  J('a_arm', 'revolute', 'base', 'a1'),
  J('a_arm_2', 'revolute', 'a1', 'a2')
])).status, 'already', 'fixed-only sibling declared ahead of the chain: already fine');

/* ... and when its siblings do have to move, it keeps its slot. */
const keepSlot = rec(modelOf([
  J('z_camera_mount', 'fixed', 'base', 'camera'),
  J('c_arm', 'revolute', 'base', 'a1'),
  J('c_arm_2', 'revolute', 'a1', 'a2'),
  J('a_wheel', 'continuous', 'base', 'w1')
]));
eq(keepSlot.status, 'found', 'fixed-only sibling with misordered neighbours: found');
eq(keepSlot.order, ['z_camera_mount', 'a_wheel', 'c_arm', 'c_arm_2'],
  'fixed-only sibling keeps its slot; only the movable branches are sorted');

/* Leaves before the deep branch, but declared after it: reorder fixes it. */
const leavesLast = rec(modelOf([
  J('c_arm', 'revolute', 'base', 'a1'),
  J('c_arm_2', 'revolute', 'a1', 'a2'),
  J('a_wheel', 'continuous', 'base', 'w1'),
  J('b_wheel', 'continuous', 'base', 'w2')
]));
eq(leavesLast.status, 'found', 'leaf siblings that sort first: found');
eq(leavesLast.order, ['a_wheel', 'b_wheel', 'c_arm', 'c_arm_2'],
  'leaf siblings that sort first: wheels, then the arm that keeps going');

/* A fixed joint between the split and the rest of the branch still counts as
   "keeps going" — BFS reaches that branch's movable joint a level later. */
const viaFixed = rec(modelOf([
  J('a_mount', 'fixed', 'base', 'm'),
  J('a_tool', 'revolute', 'm', 't'),
  J('b_leaf', 'revolute', 'base', 'x'),
  J('c_leaf', 'revolute', 'base', 'y')
]));
eq(viaFixed.status, 'none', 'branch that keeps going through a fixed joint, sorted first: none');
eq([viaFixed.split.forcedBy, viaFixed.split.deep, viaFixed.split.next], ['alphabetical', 'a_mount', 'b_leaf'],
  'branch through a fixed joint: explained as Gazebo forcing it ahead of b_leaf');

/* Floating base hanging off a world link: listed by the MuJoCo-family columns
   only, and never what decides the order. */
const floating = rec(modelOf([
  J('j2', 'revolute', 'l1', 'l2'),
  J('base_joint', 'floating', 'world', 'base'),
  J('j1', 'revolute', 'base', 'l1')
]));
eq(floating.status, 'found', 'floating base declared mid-file: found');
eq(floating.order, ['base_joint', 'j1', 'j2'], 'floating base: comes first, then the chain');

/* Two ros2_control blocks whose joints interleave in the unified order:
   entries cannot cross blocks, so only the tag mode stays off. */
const blocks = rec(modelOf([
  J('j1', 'revolute', 'base', 'l1'),
  J('j2', 'revolute', 'l1', 'l2'),
  J('j3', 'revolute', 'l2', 'l3')
], {
  ros2ControlBlocks: [{ name: 'A', type: 'system', joints: ['j1', 'j3'] }, { name: 'B', type: 'system', joints: ['j2'] }],
  ros2ControlJoints: ['j1', 'j3', 'j2']
}));
eq([blocks.status, blocks.tagModeOk], ['found', false],
  'interleaved ros2_control blocks: found, but the tag-order mode cannot follow');

const blockOrder = rec(modelOf([
  J('j1', 'revolute', 'base', 'l1'),
  J('j2', 'revolute', 'l1', 'l2')
], {
  ros2ControlBlocks: [{ name: 'A', type: 'system', joints: ['j2', 'ghost', 'j1'] }],
  ros2ControlJoints: ['j2', 'ghost', 'j1']
}));
eq([blockOrder.status, blockOrder.moved, blockOrder.ros2Changed], ['found', 0, 2],
  'only the ros2_control tag is out of order: no <joint> moves');
eq(blockOrder.model.ros2ControlBlocks[0].joints, ['j1', 'ghost', 'j2'],
  'a ros2_control entry the URDF does not define keeps its slot');

/* Refusals */
eq(Unify.recommend({ format: 'mjcf' }, null), { status: 'na', reason: 'mjcf' }, 'MJCF input: not applicable');
eq(rec(modelOf([J('j1', 'revolute', 'a', 'b')], { looksLikeXacro: true })).reason, 'xacro', 'xacro input: not applicable');
const loop = rec(modelOf([
  J('j1', 'revolute', 'base', 'a'),
  J('j2', 'revolute', 'base', 'b'),
  J('j3', 'revolute', 'a', 'c'),
  J('j4', 'revolute', 'b', 'c')
]));
eq([loop.status, loop.reason, loop.list], ['na', 'closedLoop', ['c']], 'link with two parents: refused as a closed loop');
const dup = modelOf([J('j', 'revolute', 'base', 'a'), J('j', 'revolute', 'a', 'b')]);
eq(Unify.recommend(dup, URDF.buildTree(dup)).reason, 'duplicate', 'duplicate joint names: refused');
eq(rec(modelOf([J('j1', 'revolute', 'a', 'b'), J('j2', 'revolute', 'b', 'a')])).reason, 'cycle',
  'cyclic tree: refused');

/* ── Scanner ── */
const tricky = [
  '<?xml version="1.0"?>',
  '<!DOCTYPE robot [ <!ENTITY pi "3.14"> ]>',
  '<robot name="r">',
  '  <!-- <joint name="commented_out" type="fixed"/> -->',
  '  <link name="a"><visual><![CDATA[ <joint name="cdata"/> ]]></visual></link>',
  '  <link name="b"/>',
  '  <link name="c"/>',
  '  <joint name="j_b" type="revolute" note="a > b"><parent link="a"/><child link="b"/></joint>',
  "  <joint name='j_a' type='revolute'><parent link='a'/><child link='c'/></joint>",
  '  <transmission name="t"><joint name="j_b"><hardwareInterface>x</hardwareInterface></joint></transmission>',
  '  <gazebo reference="j_a"><joint name="not_a_joint"/></gazebo>',
  '</robot>',
  ''
].join('\n');
const lite = parseLite(tricky);
eq(lite.joints.map((j) => j.name), ['j_b', 'j_a'],
  'scanner: skips comments, CDATA, DOCTYPE, quoted ">" and nested <joint> under transmission / gazebo');
const tr = pipeline(tricky);
eq(tr.rec.order, ['j_a', 'j_b'], 'scanner: two leaf siblings get Gazebo\'s alphabetical order');
eq(tr.parsed.joints.map((j) => j.name), ['j_a', 'j_b'], 'scanner: rewrite swaps the two spans');
eq(tr.rewritten.includes("<joint name='j_a' type='revolute'>") && tr.rewritten.includes('note="a > b"'), true,
  'scanner: quoting and attributes of the moved elements are kept verbatim');
eq(skeleton(tr.rewritten), skeleton(tricky), 'scanner: comments, transmission and gazebo blocks are untouched');

throws(() => Unify.scanXml('<robot><joint></robot>'), 'scanner: mismatched tags throw');
throws(() => Unify.scanXml('<robot><joint name="x"'), 'scanner: unterminated tag throws');
eq(Unify.attrOf('<joint name=" a&amp;b\tc " />', { start: 0, tagEnd: 26 }, 'name'), 'a&b c',
  'attrOf: decodes entities and normalises whitespace like the DOM');

/* The rewrite refuses text that does not line up with the model. */
const agModel = parseLite(sample('arm_gripper'));
const agRec = Unify.recommend(agModel, URDF.buildTree(agModel));
const shortModel = Object.assign({}, agModel, { joints: agModel.joints.slice(1) });
throws(() => Unify.rewriteUrdf(sample('arm_gripper'), shortModel, agRec), 'rewrite: joint count mismatch throws');
const renamed = sample('arm_gripper').replace('name="finger_right_joint" type', 'name="finger_r" type');
throws(() => Unify.rewriteUrdf(renamed, agModel, agRec), 'rewrite: joint name mismatch throws');

/* Verification catches a tampered rewrite. */
const tampered = parseLite(ag.rewritten);
tampered.joints[0] = Object.assign({}, tampered.joints[0], { parent: 'link_3' });
eq(Unify.verifyRewrite(agModel, tampered, agRec), false, 'verify: a changed parent link fails');
const reordered = parseLite(ag.rewritten);
reordered.joints.reverse();
eq(Unify.verifyRewrite(agModel, reordered, agRec), false, 'verify: a different joint order fails');

out.push('');
out.push(failed ? failed + ' failed' : 'ALL PASSED');
process.stdout.write(out.join('\n') + '\n');
if (failed) process.exit(1);
