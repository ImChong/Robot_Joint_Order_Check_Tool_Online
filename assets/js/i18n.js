/* i18n — zh (default) / en */
(function (global) {
  'use strict';

  var DICT = {
    zh: {
      'app.title': '在线机器人关节顺序检查工具',
      'app.subtitle': '比较 URDF / MJCF 在各仿真器 / 框架中的关节顺序',
      'ui.theme': '切换主题',

      'intro.title': '同一个模型，不同框架，未必是同一个关节顺序',
      'intro.lead': 'URDF 本身<b>不定义关节顺序</b>，每个框架按自己的规则排：Isaac Gym 深度优先，Isaac Sim / Isaac Lab 广度优先，Gazebo 同层还要按关节名字母序，PyBullet 连 fixed 关节一起编号。',
      'intro.demoLabel': '同一个四足 URDF，两种导入顺序',
      'intro.note': '把 Isaac Gym 训好的策略搬到 Isaac Lab 或实机上，关节向量就会<b>静默错位</b> —— 不报错，机器人直接抽搐。MJCF 也躲不掉：body 树定死了关节顺序，但 <code>&lt;actuator&gt;</code>（<code>data.ctrl</code>）是另一个向量。',
      'intro.does': '这个页面把 URDF / MJCF 在各框架里的关节顺序并列排开：一致显示绿色，对不上就标红到具体哪一格，能统一时直接给出一份调整好关节顺序的 URDF，不能统一就生成可直接粘贴的索引重映射数组。文件只在你的浏览器里解析。',
      'intro.linkRules': '各框架的顺序规则与出处 ↓',

      'input.title': '1. 载入 URDF / MJCF',
      'input.hint': '文件完全在你的浏览器本地解析，不会上传到任何服务器。',
      'input.drop': '拖拽 .urdf / .xml 文件到这里，或点击选择文件',
      'input.dropSmall': 'URDF（.urdf）或 MuJoCo MJCF（.xml），自动识别；xacro 需先展开',
      'input.samples': '示例：',
      'input.paste': '或粘贴 URDF / XML 文本 ▾',
      'input.pasteHide': '收起粘贴框 ▴',
      'input.placeholder': '在此粘贴 URDF 或 MJCF XML …',
      'input.parse': '解析',
      'input.pasted': '粘贴的文本',
      'input.upstream': '上游原始文件',

      'opts.title': '2. 选项',
      'opts.reference': '参照顺序',
      'opts.ros2c': 'ros2_control 顺序来源',
      'opts.ros2c.urdf': 'URDF 顺序 (use_urdf_to_filter=true，默认)',
      'opts.ros2c.tag': '<ros2_control> 标签顺序 (resource manager)',
      'opts.sibling': '同层子关节排序',
      'opts.sibling.auto': '按各框架自身规则（默认）',
      'opts.sibling.doc': '全部按 URDF 文档顺序',
      'opts.sibling.alpha': '全部按关节名字母序',
      'opts.showFixed': '显示 fixed 关节',

      'table.title': '3. 顺序对照表',
      'table.byIndex': '按序号',
      'table.byJoint': '按关节',
      'table.legendOk': '与参照一致',
      'table.legendBad': '与参照不一致',
      'table.legendShift': '顺序一致但因关节集合不同而错位',
      'table.legendNa': '该框架不含此关节',
      'table.idx': '序号',
      'table.joint': '关节',
      'table.type': '类型',
      'table.dof': 'DOF',
      'table.chain': '父 → 子',

      'export.title': '5. 导出与索引重映射',
      'export.hint': '生成把一个框架的关节向量重排到另一个框架的索引数组，直接贴进代码里用。',
      'export.from': '源顺序（你手上的数据）',
      'export.to': '目标顺序（你要的顺序）',
      'export.copy': '复制',
      'export.copied': '已复制 ✓',
      'export.csv': '下载 CSV',
      'export.json': '下载完整 JSON',

      'tree.title': '6. 运动学树与关节详情',
      'tree.sub': '运动学树',
      'tree.expand': '展开全部 ▾',
      'tree.collapse': '收起 ▴',
      'tree.jointsSub': '关节列表',

      'rules.title': '各框架的顺序规则',
      'rules.hint': '这些规则决定了上面的计算结果。点击展开查看依据、注意事项、运行时核对命令，以及各框架根节点四元数的获取方式与分量顺序。',
      'rules.verify': '运行时核对',
      'rules.quat': '根节点四元数',

      'verdict.ok.title': '顺序一致',
      'verdict.ok.desc': '在共有的 {n} 个关节上，所有被比较的框架顺序完全相同。',
      'verdict.bad.title': '顺序不一致',
      'verdict.bad.desc': '有 {n} 个框架的关节顺序与参照（{ref}）不同，直接把向量传过去会错位：',
      'verdict.warn.title': '顺序一致，但关节集合不同',
      'verdict.warn.desc': '各框架在共有关节上的相对顺序一致，但包含的关节不完全相同：',
      'verdict.diffCount': '{n} 个位置不同',
      'verdict.missing': '缺少 {n} 个关节：{list}',
      'verdict.extra': '多出 {n} 个关节：{list}',
      'verdict.single': '只有一个可比较的顺序，无法进行比较。',

      'unify.title': '4. 统一关节顺序',
      'unify.hint': '只调整 &lt;joint&gt; 在文件中的先后（必要时连同 &lt;ros2_control&gt; 里的关节条目），尝试找出一种写法，让同一份 URDF 在上表所有框架中得到同一个关节顺序。关节名、父子关系、类型和参数一概不动，运动学链保持原样。',
      'unify.already.title': '已经统一，无需调整',
      'unify.already.desc': '这份 URDF 在上表所有框架中的关节顺序已经一致；勾选「显示 fixed 关节」或切换 ros2_control 顺序来源也不会变。',
      'unify.found.title': '可以统一：只需重排关节',
      'unify.found.joints': '把 {n} 个 &lt;joint&gt; 移到新位置',
      'unify.found.ros2': '把 &lt;ros2_control&gt; 里的 {n} 个关节条目排成同一顺序',
      'unify.found.join': '，并',
      'unify.found.desc': '{actions}后，上表所有框架都会得到同一个关节顺序（{m} 个可动关节）。运动学链不变，文件其余内容一个字节都不改。',
      'unify.found.before': '调整前与文件顺序不一致的列：{list}',
      'unify.found.tagNote': '文件里有多个 &lt;ros2_control&gt; 块，关节条目不能跨块移动，所以按「&lt;ros2_control&gt; 标签顺序」时仍会不一致；默认的 use_urdf_to_filter=true 已经一致。',
      'unify.qual.fixed': '（计入 fixed 关节时）',
      'unify.qual.tag': '（按 &lt;ros2_control&gt; 标签顺序时）',
      'unify.download': '下载调整后的 URDF',
      'unify.reload': '载入调整后的文件并核对',
      'unify.col.idx': '统一下标',
      'unify.col.pos': '文件中位置（原 → 新）',
      'unify.fileError': '推荐顺序已经算出，但没能在原文里可靠地定位并替换每个 &lt;joint&gt;（或替换后的文件没通过自检）。为免给出损坏的文件，这里不提供下载，可以照上表手动调整。',
      'unify.none.title': '没有可统一的关节顺序',
      'unify.none.desc': '无论怎样排列 &lt;joint&gt;，深度优先（Isaac Gym / MuJoCo / Genesis / Newton / Gazebo / PyBullet）和广度优先（Isaac Sim / Isaac Lab）都给不出同一个顺序。',
      'unify.none.where': '分歧从下标 {i} 开始：深度优先排 <code>{dfs}</code>，广度优先排 <code>{bfs}</code>，两者在 <code>{link}</code> 下分属不同支链。',
      'unify.none.whereNoLink': '分歧从下标 {i} 开始：深度优先排 <code>{dfs}</code>，广度优先排 <code>{bfs}</code>。',
      'unify.none.branches': '<code>{link}</code> 下有 {k} 条支链在分叉后还有可动关节：深度优先把一条走到底再换下一条，广度优先先把每条支链的同一层排完 —— 只要存在这样的分叉，两者就注定不同。',
      'unify.none.alpha': '<code>{link}</code> 下只有 <code>{deep}</code> 这一条支链还会往下延伸，把它写在同层最后本可以让深度优先和广度优先一致；但 Gazebo 同层按关节名字母序排，会把它放到 <code>{next}</code> 前面。不改关节名就无法统一。',
      'unify.none.generic': '仍不一致的框架：{list}。',
      'unify.none.advice': '这种情况请保留原文件，用下方「5. 导出与索引重映射」在代码里转换关节下标。',
      'unify.na.title': '无法给出推荐',
      'unify.na.mjcf': '这一步只针对 URDF。MJCF 的关节顺序由 body 嵌套写死，要改只能挪动整棵 body 子树并同步 &lt;actuator&gt;，这里不做自动改写。',
      'unify.na.xacro': '检测到未展开的 xacro，先展开成纯 URDF 再来生成统一顺序。',
      'unify.na.cycle': '运动学树里有环或没连上的关节，无法在不破坏关节链的前提下给出推荐。',
      'unify.na.closedLoop': '{list} 有不止一个父关节（闭环）。调换顺序会改变哪个关节被当作树边，因此不给推荐。',
      'unify.na.duplicate': '存在重名关节：{list}。无法唯一确定每个关节该放在哪，因此不给推荐。',
      'verdict.unify.found': '只重排 &lt;joint&gt; 就能让所有框架一致 ↓',
      'verdict.unify.none': '这份 URDF 无法靠重排关节统一顺序 ↓',

      'msg.xacro': '这看起来是未展开的 xacro 文件（含 &lt;xacro:*&gt; 标签或 ${} 表达式）。请先运行 <code>xacro robot.urdf.xacro &gt; robot.urdf</code>，再上传展开后的 URDF。',
      'msg.xmlError': 'XML 解析失败：',
      'msg.noRobot': '根元素不是 &lt;robot&gt;（URDF）也不是 &lt;mujoco&gt;（MJCF），无法识别这个文件。',
      'msg.noRoot': '根元素不是 &lt;mujoco&gt;，这不是一个有效的 MJCF 文件。',
      'msg.noJoints': '文件里没有找到 &lt;joint&gt; 元素。',
      'msg.multiRoot': '发现 {n} 个根链接（{list}）。标准 URDF 应当只有一个根，这里按文档顺序依次遍历每棵树。',
      'msg.cycle': '运动学树中存在环或未连接的关节（{n} 个关节没有被遍历到）：{list}',
      'msg.undeclaredLink': '{n} 个链接只在 &lt;joint&gt; 里被引用、没有 &lt;link&gt; 定义：{list}',
      'msg.mimic': '{n} 个 mimic 关节：{list}。MuJoCo 的 URDF 导入会忽略 &lt;mimic&gt;，Isaac 各版本支持程度也不同，务必单独确认。',
      'msg.noRos2Control': '未找到 &lt;ros2_control&gt; 标签，因此不参与比较。',
      'msg.multiDof': '存在多自由度关节（floating / planar / ball）：{list}。各框架展开成的 DOF 数量和排列方式不同，关节级顺序一致不代表 DOF 级一致。',
      'msg.parsed': '已解析 <b>{name}</b> — {links} 个 link，{joints} 个 joint（{movable} 个可动）。',
      'msg.parsedMjcf': '已解析 MJCF <b>{name}</b> — {links} 个 body，{joints} 个 joint，{actuators} 个 actuator。',
      'msg.freeBase': '检测到浮动基座关节：{list}。MuJoCo / Genesis / Newton 会为它生成一个 free joint 排在关节向量最前面（占 7 个 qpos / 6 个 qvel），而 Isaac / Gazebo / PyBullet / ros2_control 把它当作自由浮动的根、根本不放进关节列表 —— 所以本工具只在前面那几列里保留它。',
      'msg.mjcfInclude': '文件里有 {n} 处 &lt;include&gt;（{list}）。被引入的 body 和 actuator 本工具读不到，下面的顺序只反映当前这个文件。',
      'msg.mjcfEquality': '{n} 个 joint 等式约束（MuJoCo 版的 mimic）：{list}。它们不改变关节顺序，但被约束的关节不是独立自由度。',
      'msg.mjcfActuatorSkipped': '{n} 个执行器没有对应到本文件里的关节（驱动 tendon / site / body，或引用了 &lt;include&gt; 里的关节）：{list}。它们同样占用 ctrl 槽位，所以「MuJoCo ctrl」一列的序号会小于真实的 ctrl 下标。',
      'msg.duplicateName': '{n} 个重名的 body / joint：{list}。MuJoCo 编译时会直接报错，这里给重复项加了 <code>#n</code> 后缀以便继续分析。',
      'msg.sampleFetch': '载入示例失败（{file}：{detail}）。这个示例存放在 <code>samples/</code> 目录里、点击时才读取，用 <code>file://</code> 直接打开页面时会被浏览器拦截 —— 请改用本地服务器（<code>python3 -m http.server</code>），或下载 <a href="{url}" target="_blank" rel="noopener">上游原始文件</a> 后拖进来。',

      'na.noTag': '无 <ros2_control> 标签',
      'na.noActuator': '无 <actuator>',
      'na.urdfOnly': '不支持 MJCF',
      'chip.ok': '一致',
      'chip.bad': '不一致',
      'chip.warn': '集合不同',
      'chip.ref': '参照',
      'chip.na': '不适用',

      'footer.disclaimer': '静态推导结果仅供排查，请以运行时打印的关节名列表为准。',
      'footer.author': '刘冲'
    },

    en: {
      'app.title': 'Robot Joint Order Check Tool Online',
      'app.subtitle': 'Compare URDF / MJCF joint ordering across simulators and frameworks',
      'ui.theme': 'Toggle theme',

      'intro.title': 'One model, different frameworks — not necessarily one joint order',
      'intro.lead': 'A URDF <b>defines no joint order of its own</b> — every framework imposes one by its own rule: Isaac Gym depth-first, Isaac Sim / Isaac Lab breadth-first, Gazebo depth-first but with siblings sorted alphabetically, PyBullet numbering fixed joints along with the rest.',
      'intro.demoLabel': 'the same quadruped URDF, two import orders',
      'intro.note': 'Move a policy trained in Isaac Gym to Isaac Lab or to real hardware and the joint vector <b>silently mismatches</b> — nothing errors, the robot just convulses. MJCF is not exempt: its body tree pins the joint order down, but <code>&lt;actuator&gt;</code> (<code>data.ctrl</code>) is a separate vector.',
      'intro.does': 'This page lays that order out side by side across the frameworks: green when they agree, red on the exact cells that do not. When a reordered URDF can make them all agree you get that file; when none can, a paste-ready index-remap array. Your file is parsed in your own browser.',
      'intro.linkRules': 'Ordering rule per framework, and its source ↓',

      'input.title': '1. Load a URDF or MJCF',
      'input.hint': 'The file is parsed entirely in your browser — nothing is uploaded to any server.',
      'input.drop': 'Drop a .urdf / .xml file here, or click to choose one',
      'input.dropSmall': 'URDF (.urdf) or MuJoCo MJCF (.xml), detected automatically; expand xacro first',
      'input.samples': 'Samples:',
      'input.paste': 'or paste URDF / XML text ▾',
      'input.pasteHide': 'hide paste box ▴',
      'input.placeholder': 'Paste URDF or MJCF XML here …',
      'input.parse': 'Parse',
      'input.pasted': 'pasted text',
      'input.upstream': 'upstream file',

      'opts.title': '2. Options',
      'opts.reference': 'Reference order',
      'opts.ros2c': 'ros2_control order source',
      'opts.ros2c.urdf': 'URDF order (use_urdf_to_filter=true, default)',
      'opts.ros2c.tag': '<ros2_control> tag order (resource manager)',
      'opts.sibling': 'Sibling joint order',
      'opts.sibling.auto': "Each framework's own rule (default)",
      'opts.sibling.doc': 'Force URDF document order',
      'opts.sibling.alpha': 'Force alphabetical by joint name',
      'opts.showFixed': 'Show fixed joints',

      'table.title': '3. Side-by-side order',
      'table.byIndex': 'By index',
      'table.byJoint': 'By joint',
      'table.legendOk': 'matches reference',
      'table.legendBad': 'differs from reference',
      'table.legendShift': 'same relative order, offset by a differing joint set',
      'table.legendNa': 'joint absent in this framework',
      'table.idx': 'Index',
      'table.joint': 'Joint',
      'table.type': 'Type',
      'table.dof': 'DOF',
      'table.chain': 'parent → child',

      'export.title': '5. Export & index remapping',
      'export.hint': 'Generate the index array that reorders a joint vector from one framework into another — paste it straight into your code.',
      'export.from': 'Source order (the data you have)',
      'export.to': 'Target order (the order you want)',
      'export.copy': 'Copy',
      'export.copied': 'Copied ✓',
      'export.csv': 'Download CSV',
      'export.json': 'Download full JSON',

      'tree.title': '6. Kinematic tree & joint details',
      'tree.sub': 'Kinematic tree',
      'tree.expand': 'Expand all ▾',
      'tree.collapse': 'Collapse ▴',
      'tree.jointsSub': 'Joints',

      'rules.title': 'Ordering rule for each framework',
      'rules.hint': 'These rules drive the results above. Expand for the reasoning, caveats, the command to verify at runtime, and how each framework hands you the root quaternion — w, x, y, z or x, y, z, w.',
      'rules.verify': 'Verify at runtime',
      'rules.quat': 'Root quaternion',

      'verdict.ok.title': 'Orders are consistent',
      'verdict.ok.desc': 'All compared frameworks agree on the order of the {n} shared joints.',
      'verdict.bad.title': 'Orders are inconsistent',
      'verdict.bad.desc': '{n} framework(s) order joints differently from the reference ({ref}) — passing a vector across will silently mismatch:',
      'verdict.warn.title': 'Order consistent, joint sets differ',
      'verdict.warn.desc': 'Relative order agrees on the shared joints, but the frameworks do not contain the same joints:',
      'verdict.diffCount': '{n} position(s) differ',
      'verdict.missing': 'missing {n} joint(s): {list}',
      'verdict.extra': 'extra {n} joint(s): {list}',
      'verdict.single': 'Only one comparable order — nothing to compare against.',

      'unify.title': '4. Unify the joint order',
      'unify.hint': 'Tries to find a document order for the &lt;joint&gt; elements (and, where needed, the joint entries inside &lt;ros2_control&gt;) that gives every framework in the table the same joint order. Joint names, parent/child links, types and parameters are never touched — the kinematic chain stays exactly as it is.',
      'unify.already.title': 'Already unified — nothing to change',
      'unify.already.desc': 'Every framework in the table already sees this URDF in the same joint order, and that holds with fixed joints shown and under either ros2_control order source.',
      'unify.found.title': 'Unifiable — just reorder the joints',
      'unify.found.joints': 'Moving {n} &lt;joint&gt; element(s)',
      'unify.found.ros2': 'putting {n} &lt;ros2_control&gt; joint entr(ies) in the same order',
      'unify.found.join': ', and ',
      'unify.found.desc': '{actions} gives every framework in the table the same joint order ({m} movable joints). The kinematic chain is unchanged and not one other byte of the file is touched.',
      'unify.found.before': 'Columns that disagree with the file as written: {list}',
      'unify.found.tagNote': 'This file has several &lt;ros2_control&gt; blocks and entries cannot move between them, so the "&lt;ros2_control&gt; tag order" mode will still differ; the default use_urdf_to_filter=true already agrees.',
      'unify.qual.fixed': ' (counting fixed joints)',
      'unify.qual.tag': ' (in &lt;ros2_control&gt; tag order)',
      'unify.download': 'Download the reordered URDF',
      'unify.reload': 'Load it here and re-check',
      'unify.col.idx': 'Unified index',
      'unify.col.pos': 'Position in file (old → new)',
      'unify.fileError': 'The order was found, but the &lt;joint&gt; elements could not be located and swapped in the original text reliably (or the result failed its self-check). Rather than hand you a broken file, no download is offered — reorder by hand following the table.',
      'unify.none.title': 'No unified joint order exists',
      'unify.none.desc': 'However the &lt;joint&gt; elements are arranged, depth-first (Isaac Gym / MuJoCo / Genesis / Newton / Gazebo / PyBullet) and breadth-first (Isaac Sim / Isaac Lab) cannot produce the same order.',
      'unify.none.where': 'They part ways at index {i}: depth-first puts <code>{dfs}</code> there, breadth-first puts <code>{bfs}</code>, and the two sit on different branches below <code>{link}</code>.',
      'unify.none.whereNoLink': 'They part ways at index {i}: depth-first puts <code>{dfs}</code> there, breadth-first puts <code>{bfs}</code>.',
      'unify.none.branches': '{k} branches below <code>{link}</code> still have movable joints past the split. Depth-first finishes one branch before starting the next, breadth-first takes every branch one level at a time — with a fork like that, the two can never agree.',
      'unify.none.alpha': 'Only <code>{deep}</code> keeps going below <code>{link}</code>, so declaring it last among its siblings would have made depth-first and breadth-first agree — but Gazebo sorts siblings by joint name and puts it ahead of <code>{next}</code>. Not fixable without renaming joints.',
      'unify.none.generic': 'Frameworks still disagreeing: {list}.',
      'unify.none.advice': 'Keep the file as it is and convert indices in code with "5. Export &amp; index remapping" below.',
      'unify.na.title': 'No recommendation',
      'unify.na.mjcf': 'This step is for URDF only. An MJCF pins its joint order to the body nesting; changing it means moving whole body subtrees and keeping &lt;actuator&gt; in step, which this page does not rewrite automatically.',
      'unify.na.xacro': 'This looks like unexpanded xacro — expand it to a plain URDF first.',
      'unify.na.cycle': 'The kinematic tree has a cycle or disconnected joints, so no order can be recommended without breaking the chain.',
      'unify.na.closedLoop': '{list} has more than one parent joint (a closed loop). Reordering would change which joint becomes the tree edge, so no recommendation is made.',
      'unify.na.duplicate': 'Duplicate joint names: {list}. Each joint\'s position cannot be pinned down, so no recommendation is made.',
      'verdict.unify.found': 'Reordering &lt;joint&gt; alone can make every framework agree ↓',
      'verdict.unify.none': 'No &lt;joint&gt; order can unify this URDF ↓',

      'msg.xacro': 'This looks like an unexpanded xacro file (it contains &lt;xacro:*&gt; tags or ${} expressions). Run <code>xacro robot.urdf.xacro &gt; robot.urdf</code> first and upload the expanded URDF.',
      'msg.xmlError': 'XML parse error: ',
      'msg.noRobot': 'The root element is neither &lt;robot&gt; (URDF) nor &lt;mujoco&gt; (MJCF) — this file is not recognised.',
      'msg.noRoot': 'The root element is not &lt;mujoco&gt; — this is not a valid MJCF file.',
      'msg.noJoints': 'No &lt;joint&gt; elements found in this file.',
      'msg.multiRoot': 'Found {n} root links ({list}). A standard URDF has exactly one root; each tree is traversed in document order here.',
      'msg.cycle': 'The kinematic tree contains a cycle or disconnected joints ({n} joint(s) never reached): {list}',
      'msg.undeclaredLink': '{n} link(s) are referenced by joints but never declared with &lt;link&gt;: {list}',
      'msg.mimic': '{n} mimic joint(s): {list}. MuJoCo\'s URDF importer ignores &lt;mimic&gt;, and Isaac support varies by version — verify these separately.',
      'msg.noRos2Control': 'No &lt;ros2_control&gt; tag found, so it is excluded from the comparison.',
      'msg.multiDof': 'Multi-DOF joints present (floating / planar / ball): {list}. Frameworks expand these into different numbers of DOFs, so matching joint order does not imply matching DOF order.',
      'msg.parsed': 'Parsed <b>{name}</b> — {links} links, {joints} joints ({movable} movable).',
      'msg.parsedMjcf': 'Parsed MJCF <b>{name}</b> — {links} bodies, {joints} joints, {actuators} actuators.',
      'msg.freeBase': 'Floating-base joint detected: {list}. MuJoCo, Genesis and Newton turn it into a free joint at the head of the vector (7 qpos / 6 qvel), while Isaac / Gazebo / PyBullet / ros2_control treat it as a free-floating root and never list it as a joint — so this tool keeps it in the former columns only.',
      'msg.mjcfInclude': '{n} &lt;include&gt; element(s) in this file ({list}). The included bodies and actuators cannot be read here, so the orders below reflect this file alone.',
      'msg.mjcfEquality': '{n} joint equality constraint(s) — MuJoCo\'s answer to &lt;mimic&gt;: {list}. They do not change joint order, but the constrained joints are not independent DOFs.',
      'msg.mjcfActuatorSkipped': '{n} actuator(s) map to no joint in this file (they drive a tendon / site / body, or a joint from an &lt;include&gt;): {list}. They still occupy a ctrl slot, so the indices in the "MuJoCo ctrl" column run lower than the real ctrl indices.',
      'msg.duplicateName': '{n} duplicate body / joint name(s): {list}. MuJoCo rejects these at compile time; a <code>#n</code> suffix was added here so the rest of the analysis can continue.',
      'msg.sampleFetch': 'Could not load the sample ({file}: {detail}). This one lives in <code>samples/</code> and is fetched on click, which the browser blocks when the page is opened over <code>file://</code> — serve it locally (<code>python3 -m http.server</code>) or download the <a href="{url}" target="_blank" rel="noopener">upstream file</a> and drop it in.',

      'na.noTag': 'no <ros2_control> tag',
      'na.noActuator': 'no <actuator>',
      'na.urdfOnly': 'no MJCF support',
      'chip.ok': 'match',
      'chip.bad': 'differs',
      'chip.warn': 'set differs',
      'chip.ref': 'reference',
      'chip.na': 'n/a',

      'footer.disclaimer': 'Statically derived — for triage only. Trust the joint names your runtime prints.',
      'footer.author': 'Chong Liu'
    }
  };

  var lang = 'zh';

  function t(key, vars) {
    var s = (DICT[lang] && DICT[lang][key]) || (DICT.zh && DICT.zh[key]) || key;
    if (vars) {
      Object.keys(vars).forEach(function (k) {
        s = s.split('{' + k + '}').join(vars[k]);
      });
    }
    return s;
  }

  function setLang(next) {
    lang = DICT[next] ? next : 'zh';
    document.documentElement.lang = lang === 'zh' ? 'zh-CN' : 'en';
    applyStatic();
  }

  function applyStatic(root) {
    var scope = root || document;
    scope.querySelectorAll('[data-i18n]').forEach(function (el) {
      el.textContent = t(el.getAttribute('data-i18n'));
    });
    // Only for strings that carry markup of their own (<b>, <code>) — the
    // source is this dictionary, never user input.
    scope.querySelectorAll('[data-i18n-html]').forEach(function (el) {
      el.innerHTML = t(el.getAttribute('data-i18n-html'));
    });
    scope.querySelectorAll('[data-i18n-title]').forEach(function (el) {
      el.title = t(el.getAttribute('data-i18n-title'));
    });
    scope.querySelectorAll('[data-i18n-placeholder]').forEach(function (el) {
      el.placeholder = t(el.getAttribute('data-i18n-placeholder'));
    });
  }

  global.I18N = {
    t: t,
    setLang: setLang,
    getLang: function () { return lang; },
    applyStatic: applyStatic
  };
})(window);
