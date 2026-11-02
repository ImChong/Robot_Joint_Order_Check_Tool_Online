/* Recommend a <joint> order that every framework agrees on, and write it back
   into the URDF text without touching anything else.

   Only the document order of the <joint> elements (and of the <joint> entries
   inside <ros2_control>) is free to change: names, parent/child links, types
   and parameters stay as they are, so the kinematic chain is untouched.

   Why one candidate is enough
   ---------------------------
   The file order can be anything, so it is set to the depth-first order of the
   tree; that alone satisfies every column that walks the tree depth-first with
   document-order siblings (Isaac Gym, MuJoCo, Genesis, Newton, PyBullet) and
   every column that follows the file (ros2_control by default). What is left:

   - Gazebo sorts siblings by joint name. At each link, the children whose
     subtrees contain a movable joint therefore have exactly one admissible
     order — alphabetical. Children that only carry fixed joints never show up
     in a movable vector, so they keep their slots.
   - Isaac Sim / Isaac Lab walk breadth-first. With the sibling order pinned by
     Gazebo there is nothing left to choose, so the candidate either matches the
     breadth-first order or no <joint> arrangement can.

   So the candidate is built once and checked with the same analysis the table
   uses; if it fails, the answer is "no unified order exists".             */
(function (global) {
  'use strict';

  var URDF = global.URDF;
  var Orderings = global.Orderings;

  /* Same comparator the Gazebo column uses for its alphabetical siblings. */
  function byName(a, b) { return a.name < b.name ? -1 : a.name > b.name ? 1 : 0; }

  /* Every setting that changes which joints a column holds. A unified order
     has to survive all of them, not just the one currently on screen. */
  var COMBOS = [
    { showFixed: false, ros2cMode: 'urdf' },
    { showFixed: true, ros2cMode: 'urdf' },
    { showFixed: false, ros2cMode: 'tag' },
    { showFixed: true, ros2cMode: 'tag' }
  ];

  /** Columns that disagree with the file order, under each setting. */
  function conflicts(model, tree) {
    var out = [];
    COMBOS.forEach(function (c) {
      // Without a <ros2_control> tag the two modes compute the same thing.
      if (c.ros2cMode === 'tag' && !model.ros2ControlJoints.length) return;
      var a = Orderings.analyze(model, tree, {
        refId: 'file', siblingOrder: 'auto', ros2cMode: c.ros2cMode, showFixed: c.showFixed
      });
      a.badColumns.forEach(function (col) {
        out.push({ id: col.id, showFixed: c.showFixed, ros2cMode: c.ros2cMode });
      });
    });
    return out;
  }

  function counted(list) {
    var seen = Object.create(null), dup = [];
    list.forEach(function (n) {
      if (seen[n] === 1) dup.push(n);
      seen[n] = (seen[n] || 0) + 1;
    });
    return dup;
  }

  /* The joints a framework puts in its movable vector: movable, and not the
     floating base that only the MuJoCo-family columns keep. */
  function vectorFilter(model, tree) {
    var freeBase = Object.create(null);
    Orderings.freeBaseJoints(model, tree).forEach(function (j) { freeBase[j.name] = true; });
    return function (j) { return URDF.isMovable(j) && !freeBase[j.name]; };
  }

  /** A copy of `model` with its joints (and ros2_control entries) in `order`. */
  function reorderModel(model, order) {
    var rank = Object.create(null);
    order.forEach(function (j, i) { rank[j.name] = i; });

    var joints = order.map(function (j, i) {
      var copy = {};
      Object.keys(j).forEach(function (k) { copy[k] = j[k]; });
      copy.docIndex = i;
      return copy;
    });
    var jointByName = {};
    joints.forEach(function (j) { jointByName[j.name] = j; });

    var blocks = model.ros2ControlBlocks.map(function (b) {
      return { name: b.name, type: b.type, joints: sortSlots(b.joints, rank) };
    });
    var union = [];
    blocks.forEach(function (b) {
      b.joints.forEach(function (n) { if (union.indexOf(n) === -1) union.push(n); });
    });

    var out = {};
    Object.keys(model).forEach(function (k) { out[k] = model[k]; });
    out.joints = joints;
    out.jointByName = jointByName;
    out.ros2ControlBlocks = blocks;
    out.ros2ControlJoints = union;
    return out;
  }

  /* Reorder the names that have a rank into the slots they already occupy;
     anything unranked (a name the URDF does not define) stays where it is. */
  function sortSlots(names, rank) {
    var known = names.filter(function (n) { return n in rank; });
    known.sort(function (a, b) { return rank[a] - rank[b]; });
    var k = 0;
    return names.map(function (n) { return n in rank ? known[k++] : n; });
  }

  /* Links from `link` up to its root, nearest first. */
  function linkPath(tree, link) {
    var path = [link], guard = 0;
    while (tree.parentJoint[link] && guard++ < 100000) {
      link = tree.parentJoint[link].parent;
      path.push(link);
    }
    return path;
  }

  /**
   * Where depth-first and breadth-first part ways, phrased for the UI:
   * the first index that differs, the two joints, the link they branch from,
   * and whether the split is structural (several branches keep going) or only
   * forced by Gazebo's alphabetical siblings.
   */
  function explainSplit(model, tree, carries) {
    var inVector = vectorFilter(model, tree);
    var dfs = Orderings.dfsOrder(tree, 'document').filter(inVector);
    var bfs = Orderings.bfsOrder(tree, 'document').filter(inVector);
    var i = 0;
    while (i < dfs.length && i < bfs.length && dfs[i] === bfs[i]) i++;
    if (i >= dfs.length || i >= bfs.length) return null;

    var a = dfs[i], b = bfs[i];
    var pa = linkPath(tree, a.child), pb = linkPath(tree, b.child);
    var onB = Object.create(null);
    pb.forEach(function (l) { onB[l] = true; });
    var at = -1;
    for (var k = 0; k < pa.length; k++) { if (onB[pa[k]]) { at = k; break; } }

    var out = { index: i, dfs: a.name, bfs: b.name, link: null };
    if (at <= 0) return out;   // separate roots, or nothing in common
    var link = pa[at];
    out.link = link;

    // Branches under the split link that carry a movable joint, and which of
    // them still carry one below their first joint.
    var kids = (tree.childJoints[link] || []).filter(carries);
    var deeper = kids.filter(function (j) {
      return (tree.childJoints[j.child] || []).some(carries);
    });
    out.branches = deeper.length;
    if (deeper.length === 1 && kids[kids.length - 1] !== deeper[0]) {
      out.forcedBy = 'alphabetical';
      out.deep = deeper[0].name;
      out.next = kids[kids.indexOf(deeper[0]) + 1].name;
    } else if (deeper.length > 1) {
      out.forcedBy = 'branches';
    }
    return out;
  }

  /**
   * @returns {{status: 'na'|'already'|'found'|'none', ...}}
   *   na      — reason: 'mjcf' | 'xacro' | 'cycle' | 'closedLoop' | 'duplicate', list?
   *   already — the file already agrees everywhere
   *   found   — order (names, new document order), model (reordered), moved,
   *             movable, ros2Changed, tagModeOk, before (conflicts as written)
   *   none    — before, after (conflicts left by the best candidate), split
   */
  function recommend(model, tree) {
    if (!model || model.format !== 'urdf') return { status: 'na', reason: 'mjcf' };
    if (model.looksLikeXacro) return { status: 'na', reason: 'xacro' };
    if (!tree || !tree.roots.length || tree.warnings.some(function (w) { return w.code === 'cycle'; })) {
      return { status: 'na', reason: 'cycle' };
    }

    var dupNames = counted(model.joints.map(function (j) { return j.name; }));
    if (dupNames.length) return { status: 'na', reason: 'duplicate', list: dupNames };

    // A link with two parent joints closes a loop; which one becomes the tree
    // edge depends on document order, so moving joints would change the chain.
    var twoParents = counted(model.joints.map(function (j) { return j.child; }).filter(Boolean));
    if (twoParents.length) return { status: 'na', reason: 'closedLoop', list: twoParents };

    var before = conflicts(model, tree);
    if (!before.length) return { status: 'already' };

    var inVector = vectorFilter(model, tree);
    var memo = Object.create(null);
    function carries(j) {
      if (j.name in memo) return memo[j.name];
      memo[j.name] = false;   // re-entry guard; loops were rejected above
      var own = inVector(j);
      var below = (tree.childJoints[j.child] || []).some(carries);
      memo[j.name] = own || below;
      return memo[j.name];
    }

    var childJoints = {};
    Object.keys(tree.childJoints).forEach(function (link) {
      var kids = tree.childJoints[link];
      var slots = [], movers = [];
      kids.forEach(function (j, i) {
        if (carries(j)) { slots.push(i); movers.push(j); }
      });
      movers.sort(byName);
      var next = kids.slice();
      slots.forEach(function (s, k) { next[s] = movers[k]; });
      childJoints[link] = next;
    });

    var order = Orderings.dfsOrder({ roots: tree.roots, childJoints: childJoints }, 'document');
    if (order.length !== model.joints.length) return { status: 'na', reason: 'cycle' };

    var candidate = reorderModel(model, order);
    var candTree = URDF.buildTree(candidate);
    var after = conflicts(candidate, candTree);

    // Joints cannot move between <ros2_control> blocks (each block is its own
    // hardware interface), so with several blocks the tag order may still
    // disagree. That alone does not sink the result — the default
    // use_urdf_to_filter=true follows the file — but the UI says so.
    var hard = after.filter(function (c) { return c.ros2cMode !== 'tag'; });
    if (hard.length) {
      return {
        status: 'none',
        before: before,
        after: hard,
        split: explainSplit(candidate, candTree, carries)
      };
    }

    var moved = 0;
    order.forEach(function (j, i) { if (model.joints[i] !== j) moved++; });
    var ros2Changed = 0;
    model.ros2ControlBlocks.forEach(function (b, bi) {
      candidate.ros2ControlBlocks[bi].joints.forEach(function (n, i) {
        if (b.joints[i] !== n) ros2Changed++;
      });
    });

    return {
      status: 'found',
      order: order.map(function (j) { return j.name; }),
      model: candidate,
      tree: candTree,
      moved: moved,
      movable: order.filter(inVector).length,
      ros2Changed: ros2Changed,
      tagModeOk: !after.length,
      before: before
    };
  }

  /* ── Text rewrite ──────────────────────────────────────────────────
     A DOM round-trip would reformat the whole file (quotes, declaration,
     whitespace), so the joints are moved as raw text instead: find each
     <joint> element's span, then write the spans back in the new order into
     the same slots. Everything between the slots — links, comments,
     <gazebo>, <transmission>, formatting — stays byte-for-byte.            */

  function skipPast(text, token, from) {
    var at = text.indexOf(token, from);
    if (at === -1) throw new Error('unterminated ' + token);
    return at + token.length;
  }

  /* <!DOCTYPE …> may carry an internal subset in [ … ] with quoted strings. */
  function skipDeclaration(text, from) {
    var depth = 0, q = null;
    for (var i = from + 2; i < text.length; i++) {
      var ch = text.charAt(i);
      if (q) { if (ch === q) q = null; continue; }
      if (ch === '"' || ch === "'") q = ch;
      else if (ch === '[') depth++;
      else if (ch === ']') depth--;
      else if (ch === '>' && depth <= 0) return i + 1;
    }
    throw new Error('unterminated <!');
  }

  /**
   * Element spans of a well-formed XML document, in document order:
   * {name, start, tagEnd, end, parent, index}. `start`..`end` covers the whole
   * element; `start`..`tagEnd` its start tag. Comments, CDATA, processing
   * instructions and quoted attribute values are skipped, so a "<joint" in any
   * of them is not mistaken for an element.
   */
  function scanXml(text) {
    var els = [], stack = [], i = 0, n = text.length;
    while (i < n) {
      var lt = text.indexOf('<', i);
      if (lt === -1) break;
      if (text.substr(lt, 4) === '<!--') { i = skipPast(text, '-->', lt + 4); continue; }
      if (text.substr(lt, 9) === '<![CDATA[') { i = skipPast(text, ']]>', lt + 9); continue; }
      var next = text.charAt(lt + 1);
      if (next === '?') { i = skipPast(text, '?>', lt + 2); continue; }
      if (next === '!') { i = skipDeclaration(text, lt); continue; }

      if (next === '/') {
        var close = text.indexOf('>', lt);
        if (close === -1) throw new Error('unterminated end tag');
        var closing = text.slice(lt + 2, close).trim();
        var open = stack.pop();
        if (!open || open.name !== closing) throw new Error('mismatched </' + closing + '>');
        open.end = close + 1;
        i = close + 1;
        continue;
      }

      var j = lt + 1;
      while (j < n && !/[\s/>]/.test(text.charAt(j))) j++;
      var name = text.slice(lt + 1, j);
      var q = null;
      for (; j < n; j++) {
        var ch = text.charAt(j);
        if (q) { if (ch === q) q = null; }
        else if (ch === '"' || ch === "'") q = ch;
        else if (ch === '>') break;
      }
      if (j >= n) throw new Error('unterminated <' + name + '>');

      var el = {
        name: name,
        start: lt,
        tagEnd: j + 1,
        end: -1,
        parent: stack.length ? stack[stack.length - 1].index : -1,
        index: els.length
      };
      els.push(el);
      if (text.charAt(j - 1) === '/') el.end = j + 1;
      else stack.push(el);
      i = j + 1;
    }
    if (stack.length) throw new Error('unclosed <' + stack[stack.length - 1].name + '>');
    return els;
  }

  function localName(qname) {
    var c = qname.indexOf(':');
    return c === -1 ? qname : qname.slice(c + 1);
  }

  var ENTITIES = { lt: '<', gt: '>', amp: '&', quot: '"', apos: "'" };

  /* An attribute value as the DOM would report it (entities decoded,
     whitespace normalised), trimmed like urdf.js does. null when absent. */
  function attrOf(text, el, attr) {
    var tag = text.slice(el.start, el.tagEnd);
    var re = new RegExp('\\s' + attr + '\\s*=\\s*(?:"([^"]*)"|\'([^\']*)\')');
    var m = re.exec(tag);
    if (!m) return null;
    var raw = m[1] !== undefined ? m[1] : m[2];
    return raw.replace(/[\t\n\r]/g, ' ').replace(/&(#x[0-9a-fA-F]+|#[0-9]+|[a-zA-Z]+);/g, function (all, ent) {
      if (ent.charAt(0) === '#') {
        var code = ent.charAt(1) === 'x' ? parseInt(ent.slice(2), 16) : parseInt(ent.slice(1), 10);
        return String.fromCharCode(code);
      }
      return Object.prototype.hasOwnProperty.call(ENTITIES, ent) ? ENTITIES[ent] : all;
    }).trim();
  }

  function childrenNamed(els, parentIndex, name) {
    return els.filter(function (e) { return e.parent === parentIndex && localName(e.name) === name; });
  }

  /**
   * The URDF text with its <joint> elements (and <ros2_control> joint entries)
   * in `rec.order`. Throws when the text does not line up with the parsed
   * model — better no file than a wrong one.
   */
  function rewriteUrdf(text, model, rec) {
    var els = scanXml(text);
    var root = els.filter(function (e) { return e.parent === -1; })[0];
    if (!root || localName(root.name) !== 'robot') throw new Error('no <robot> root');

    var jointEls = childrenNamed(els, root.index, 'joint');
    if (jointEls.length !== model.joints.length) throw new Error('joint count mismatch');
    var spanOf = Object.create(null);
    jointEls.forEach(function (e, i) {
      var nm = attrOf(text, e, 'name');
      if (nm !== null && nm !== '' && nm !== model.joints[i].name) throw new Error('joint name mismatch');
      spanOf[model.joints[i].name] = text.slice(e.start, e.end);
    });

    var edits = jointEls.map(function (e, i) {
      return { start: e.start, end: e.end, text: spanOf[rec.order[i]] };
    });

    var rank = Object.create(null);
    rec.order.forEach(function (n, i) { rank[n] = i; });
    childrenNamed(els, root.index, 'ros2_control').forEach(function (block) {
      var entries = childrenNamed(els, block.index, 'joint');
      var names = entries.map(function (e, i) {
        var nm = attrOf(text, e, 'name');
        // Unnamed entries are ignored by the parser too; give them a key that
        // cannot be ranked so they keep their slot.
        return nm ? nm : '\u0000' + i;
      });
      var span = Object.create(null);
      entries.forEach(function (e, i) { span[names[i]] = text.slice(e.start, e.end); });
      sortSlots(names, rank).forEach(function (n, i) {
        edits.push({ start: entries[i].start, end: entries[i].end, text: span[n] });
      });
    });

    edits.sort(function (a, b) { return a.start - b.start; });
    var out = '', at = 0;
    edits.forEach(function (ed) {
      out += text.slice(at, ed.start) + ed.text;
      at = ed.end;
    });
    return out + text.slice(at);
  }

  /**
   * Re-parsed rewrite vs. the original: same links, every joint unchanged but
   * for its position, the joints in the recommended order, the same
   * ros2_control entries per block — and no framework left disagreeing.
   */
  function verifyRewrite(original, parsed, rec) {
    if (!parsed || parsed.errors.length) return false;
    var names = parsed.joints.map(function (j) { return j.name; });
    if (names.join('\n') !== rec.order.join('\n')) return false;

    var linkNames = function (m) { return m.links.map(function (l) { return l.name; }).join('\n'); };
    if (linkNames(parsed) !== linkNames(original)) return false;

    var same = parsed.joints.every(function (j) {
      var o = original.jointByName[j.name];
      return o && o.type === j.type && o.parent === j.parent && o.child === j.child && o.mimic === j.mimic;
    });
    if (!same) return false;

    if (parsed.ros2ControlBlocks.length !== original.ros2ControlBlocks.length) return false;
    var blocksOk = parsed.ros2ControlBlocks.every(function (b, i) {
      var o = original.ros2ControlBlocks[i];
      return b.name === o.name && b.joints.slice().sort().join('\n') === o.joints.slice().sort().join('\n');
    });
    if (!blocksOk) return false;

    var left = conflicts(parsed, URDF.buildTree(parsed)).filter(function (c) {
      return !(rec.tagModeOk === false && c.ros2cMode === 'tag');
    });
    return left.length === 0;
  }

  global.Unify = {
    COMBOS: COMBOS,
    conflicts: conflicts,
    recommend: recommend,
    reorderModel: reorderModel,
    scanXml: scanXml,
    attrOf: attrOf,
    rewriteUrdf: rewriteUrdf,
    verifyRewrite: verifyRewrite
  };
})(window);
