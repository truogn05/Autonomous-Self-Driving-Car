// Run the actual Road.proto JavaScript template without launching Webots.
// Covers lane-count edits and grouped endpoint paint, including empty slots.
// Usage: node webots/tools/check_road_template.cjs
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
function compileTemplate(filename) {
  const source = fs.readFileSync(path.join(__dirname,
    '../protos/vendor/projects/objects/road/protos/', filename), 'utf8');
  let code = 'let out = "";\n';
  let cursor = source.indexOf('%<');
  for (const match of source.slice(cursor).matchAll(/%<(=)?([\s\S]*?)>%/g)) {
    const index = source.indexOf('%<') + match.index;
    code += `out += ${JSON.stringify(source.slice(cursor, index))};\n`;
    code += match[1] ? `out += (${match[2]});\n`
      : match[2].replace(/import .*?;/g, '') + '\n';
    cursor = index + match[0].length;
  }
  code += `out += ${JSON.stringify(source.slice(cursor))}; return out;`;
  return new Function('fields', 'wbgeometry', 'wbvector2', 'wbvector3', code);
}
const render = compileTemplate('Road.proto');
const vector2 = {
  angle: (a, b) => Math.atan2(a.y-b.y, a.x-b.x),
  distance: (a, b) => Math.hypot(a.x-b.x, a.y-b.y)
};
const vector3 = {distance: (a, b) => Math.hypot(a.x-b.x, a.y-b.y, a.z-b.z)};
const line = type => ({node_name: 'RoadLine', fields: {
  width: {value: .025}, type: {value: type}, color: {value: {r: 1, g: 1, b: 1}}
}});
function expand(n, start, end, curved, median = false) {
  const values = {
    width: n === 8 ? 4.8 : 2.4, numberOfLanes: n, roadBorderHeight: .035, roadBorderWidth: [.14],
    lines: (n === 8 ? ['dashed','dashed','dashed','double','dashed','dashed','dashed'] : ['dashed', 'double', 'dashed']).map((type, i) => line(median && i === n/2-1 ? 'none' : type)), rightBorder: true, leftBorder: true,
    rightBarrier: false, leftBarrier: false, bottom: false, road: true,
    roadBoundingObject: false, rightBorderBoundingObject: true, leftBorderBoundingObject: true,
    startingAngle: [], endingAngle: [], startLine: start, endLine: end, bottomTexture: [],
    wayPoints: curved ? [{x:0,y:0,z:0}, {x:3,y:0,z:0}, {x:5,y:2,z:0}]
      : [{x:0,y:0,z:0}, {x:5,y:0,z:0}],
    roadTilt: [0,0], splineSubdivision: -1, turnLanesForward: '', turnLanesBackward: ''
  };
  const fields = Object.fromEntries(Object.entries(values).map(([k,v]) => [k,{value:v,defaultValue:v}]));
  return render(fields, {}, vector2, vector3);
}
for (const curved of [false, true]) {
  for (const n of [4, 8]) {
    const result = expand(n, [], [], curved, true);
    assert(!/NaN|undefined/.test(result));
    const dividers = result.split('# lines')[1].split('# right border')[0].split('DEF LINES Solid').slice(1);
    assert.equal(dividers.length, n-2, 'Explicit none must suppress only the centre slot');
    for (const block of dividers) {
      const p = points(block);
      assert(Math.abs(Math.hypot(p[3]-p[0], p[4]-p[1])-.025) < 1e-8);
      assert(block.includes('line_base_color_dashed.png'), 'Median must retain the other dashed lane dividers');
      const centerY = (p[1]+p[4])/2;
      assert(Math.abs(centerY) > .5, 'A hidden default centre line was generated');
    }
  }
}
function points(block) {
  const data = block.match(/coord Coordinate\s*\{\s*point\s*\[([^\]]+)/)[1];
  return data.trim().split(/\s+/).map(Number);
}
for (const curved of [false, true]) {
  for (const n of [4,5,6,7,8]) {
    const result = expand(n, [], [], curved);
    assert(!/NaN|undefined/.test(result));
    const dividers = result.split('# lines')[1].split('DEF LINES Solid').slice(1);
    assert.equal(dividers.length, n-1);
    dividers.forEach((block, i) => {
      const p = points(block);
      const width = Math.hypot(p[3]-p[0],p[4]-p[1]);
      const doubleIndex = n === 8 ? 3 : 1;
      assert(Math.abs(width - (i === doubleIndex ? .075 : .025)) < 1e-8);
      if (i !== doubleIndex) assert(block.includes('line_base_color_dashed.png'));
    });
  }
}
for (const [start, end, count] of [
  [['','','stop','stop'], ['stop','stop','',''], 2],
  [['cross','cross','stop_cross','stop_cross'], ['stop_cross','stop_cross','cross','cross'], 4],
  [[],[],0]
]) {
  const result = expand(4,start,end,false);
  const paint = [...result.matchAll(/geometry DEF (?:START|END)_ROAD_LINES_GEOMETRY([\s\S]*?)creaseAngle/g)];
  assert.equal(paint.length,count);
  for (const match of paint) {
    const p = points(match[1]);
    assert(Math.abs(Math.hypot(p[3]-p[0],p[4]-p[1])-1.2) < 1e-8);
  }
}
const eightPaint = expand(8, ['', '', '', '', 'stop', 'stop', 'stop', 'stop'],
  ['stop', 'stop', 'stop', 'stop', '', '', '', ''], false);
const eightBlocks = [...eightPaint.matchAll(/geometry DEF (?:START|END)_ROAD_LINES_GEOMETRY([\s\S]*?)creaseAngle/g)];
assert.equal(eightBlocks.length, 2);
for (const block of eightBlocks) {
  const p = points(block[1]);
  assert(Math.abs(Math.hypot(p[3]-p[0], p[4]-p[1])-2.4) < 1e-8);
}
const straightValues = {length: 5, startingRoadTilt: 0, endingRoadTilt: 0,
  startingRoadBorderWidth: .14, endingRoadBorderWidth: .14};
const straight = compileTemplate('StraightRoadSegment.proto')(
  Object.fromEntries(Object.entries(straightValues).map(([k,v]) => [k,{value:v,defaultValue:v}])));
for (const field of ['id', 'startJunction', 'endJunction', 'numberOfForwardLanes'])
  assert(new RegExp(`\\b${field} IS ${field}\\b`).test(straight), `StraightRoadSegment must forward ${field}`);
console.log('Road template OK: 4/5/6/7/8 lanes, straight/curved, explicit median centre slots, endpoint markings, roundabout blanks and junction bindings.');
