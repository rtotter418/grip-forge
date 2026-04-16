import zlib from 'zlib';

// ─────────────────────────────────────────────────────────────────────────────
//  GRIP FORGE  —  Geometry Engine
// ─────────────────────────────────────────────────────────────────────────────

const PROF_PTS = 48;

function superEllipse(w, h, n) {
  const a = w/2, b = h/2, e = 2/n, pts = [];
  for (let i = 0; i < PROF_PTS; i++) {
    const t  = (2*Math.PI*i)/PROF_PTS;
    const ct = Math.cos(t), st = Math.sin(t);
    pts.push([a * Math.sign(ct) * Math.pow(Math.abs(ct), e),
              b * Math.sign(st) * Math.pow(Math.abs(st), e)]);
  }
  return pts;
}

function rectProfile(w, h) {
  const a = w/2, b = h/2;
  const segs = [[a,0,a,b],[a,b,-a,b],[-a,b,-a,-b],[-a,-b,a,-b],[a,-b,a,0]];
  const lens  = segs.map(s => { const dx=s[2]-s[0],dy=s[3]-s[1]; return Math.sqrt(dx*dx+dy*dy); });
  const total = lens.reduce((a,b)=>a+b, 0);
  const pts = [];
  for (let i = 0; i < PROF_PTS; i++) {
    let tgt = (i/PROF_PTS)*total, acc = 0;
    for (let s = 0; s < segs.length; s++) {
      if (tgt <= acc + lens[s]) {
        const u = (tgt-acc)/lens[s];
        pts.push([segs[s][0]+u*(segs[s][2]-segs[s][0]),
                  segs[s][1]+u*(segs[s][3]-segs[s][1])]);
        break;
      }
      acc += lens[s];
    }
  }
  return pts;
}

function pctToN(pct) { return 2 * Math.pow(20, Math.pow(pct/100, 2.322)); }

function scalePro(prof, sx, sy) { return prof.map(p => [p[0]*sx, p[1]*sy]); }

function lerpPro(p0, p1, t) {
  return p0.map((pt, i) => [pt[0]*(1-t)+p1[i][0]*t, pt[1]*(1-t)+p1[i][1]*t]);
}

function resamplePath(path, n) {
  const nn = path.length;
  const cumLen = [0];
  for (let i = 1; i < nn; i++) {
    const dx = path[i][0]-path[i-1][0], dy = path[i][1]-path[i-1][1];
    cumLen.push(cumLen[i-1]+Math.sqrt(dx*dx+dy*dy));
  }
  const cldx = path[0][0]-path[nn-1][0], cldy = path[0][1]-path[nn-1][1];
  const total = cumLen[nn-1]+Math.sqrt(cldx*cldx+cldy*cldy);
  const pts = [];
  for (let k = 0; k < n; k++) {
    let tgt = (k/n)*total, idx = 0;
    while (idx < nn-1 && cumLen[idx+1] < tgt) idx++;
    const isClose = (idx === nn-1);
    const segLen  = isClose ? Math.sqrt(cldx*cldx+cldy*cldy) : cumLen[idx+1]-cumLen[idx];
    const u       = segLen > 1e-10 ? (tgt-cumLen[idx])/segLen : 0;
    const nx = isClose ? path[0][0] : path[idx+1][0];
    const ny = isClose ? path[0][1] : path[idx+1][1];
    pts.push([path[idx][0]+u*(nx-path[idx][0]), path[idx][1]+u*(ny-path[idx][1])]);
  }
  return pts;
}

function hexProfile(w, h, stickOut, sharpPct) {
  stickOut = Math.max(0, Math.min(stickOut, h/2*0.9));
  const flatH   = Math.max(0.02, h - 2*stickOut);
  const diagLen = Math.sqrt((w/2)*(w/2) + stickOut*stickOut);
  const maxR    = Math.min(flatH, diagLen) * 0.44;
  const r       = maxR * Math.pow(Math.max(0, 1-sharpPct/100), 2.322);
  const startPt = [w/2, 0];
  const C = [
    [ w/2,  flatH/2],
    [   0,  h/2    ],
    [-w/2,  flatH/2],
    [-w/2, -flatH/2],
    [   0, -h/2    ],
    [ w/2, -flatH/2],
  ];
  const path = [startPt];
  for (let i = 0; i < C.length; i++) {
    const prev = i === 0 ? startPt : C[i-1];
    const cur  = C[i];
    const next = i === C.length-1 ? startPt : C[i+1];
    const d0x = prev[0]-cur[0], d0y = prev[1]-cur[1];
    const d1x = next[0]-cur[0], d1y = next[1]-cur[1];
    const l0  = Math.sqrt(d0x*d0x+d0y*d0y);
    const l1  = Math.sqrt(d1x*d1x+d1y*d1y);
    const cr  = (r < 0.02||l0 < 0.02||l1 < 0.02) ? 0 : Math.min(r, l0*0.4, l1*0.4);
    if (cr < 0.02) {
      path.push([cur[0], cur[1]]);
    } else {
      const p0x = cur[0]+d0x/l0*cr, p0y = cur[1]+d0y/l0*cr;
      const p1x = cur[0]+d1x/l1*cr, p1y = cur[1]+d1y/l1*cr;
      path.push([p0x, p0y]);
      for (let j = 1; j <= 5; j++) {
        const tt = j/6;
        path.push([
          (1-tt)*(1-tt)*p0x + 2*(1-tt)*tt*cur[0] + tt*tt*p1x,
          (1-tt)*(1-tt)*p0y + 2*(1-tt)*tt*cur[1] + tt*tt*p1y
        ]);
      }
      path.push([p1x, p1y]);
    }
  }
  return resamplePath(path, PROF_PTS);
}

function makeProf(w, h, type, param, so) {
  if (type === 'hx') return hexProfile(w, h, so, param);
  return scalePro(superEllipse(1, 1, pctToN(param)), w, h);
}

function loftSurf(profs, zLvls, flip) {
  const tris = [];
  for (let i = 0; i < profs.length-1; i++) {
    const p0 = profs[i], p1 = profs[i+1];
    const z0 = zLvls[i],  z1 = zLvls[i+1];
    const n  = p0.length;
    for (let j = 0; j < n; j++) {
      const j1 = (j+1)%n;
      const a=[p0[j][0], p0[j][1], z0], b=[p0[j1][0],p0[j1][1],z0];
      const c=[p1[j1][0],p1[j1][1],z1], d=[p1[j][0], p1[j][1], z1];
      if (!flip) { tris.push([a,b,c]); tris.push([a,c,d]); }
      else       { tris.push([a,c,b]); tris.push([a,d,c]); }
    }
  }
  return tris;
}

function solidCap(pts, z, faceUp) {
  let cx=0, cy=0;
  pts.forEach(p => { cx+=p[0]; cy+=p[1]; });
  cx/=pts.length; cy/=pts.length;
  const ctr=[cx,cy,z], tris=[], n=pts.length;
  for (let j=0; j<n; j++) {
    const j1=(j+1)%n;
    const a=[pts[j][0], pts[j][1], z], b=[pts[j1][0],pts[j1][1],z];
    faceUp ? tris.push([ctr,a,b]) : tris.push([ctr,b,a]);
  }
  return tris;
}

function ringCap(oP, iP, z, faceUp) {
  const tris=[], n=oP.length;
  for (let j=0; j<n; j++) {
    const j1=(j+1)%n;
    const oa=[oP[j][0], oP[j][1], z],  ob=[oP[j1][0],oP[j1][1],z];
    const ia=[iP[j][0], iP[j][1], z],  ib=[iP[j1][0],iP[j1][1],z];
    if (faceUp) { tris.push([oa,ob,ib]); tris.push([oa,ib,ia]); }
    else        { tris.push([oa,ib,ob]); tris.push([oa,ia,ib]); }
  }
  return tris;
}

function buildGrip(p) {
  const totalLen = p.lBef + p.lAft;
  const N_SLICES = 60, MIN_WALL = 0.4;
  let clamped = false;

  const bot = { w:p.wBot, d:p.dBot, type:p.typeBot, param:p.paramBot, so:p.soBot };
  const tap = { w:p.wTap, d:p.dTap, type:p.typeTap, param:p.paramTap, so:p.soTap };
  const top = { w:p.wTop, d:p.dTop, type:p.typeTop, param:p.paramTop, so:p.soTop };

  function outerAt(z) {
    // ── No-taper mode: single segment guard→pommel ────────────────────────────
    if (p.noTaper) {
      const t = totalLen > 0 ? z / totalLen : 0;
      let w = bot.w + (top.w - bot.w) * t;
      let d = bot.d + (top.d - bot.d) * t;
      // Each bulge applies to its half of the grip
      const bulge  = t < 0.5 ? p.bulgeBef : p.bulgeAft;
      const localT = t < 0.5 ? t * 2 : (t - 0.5) * 2;
      const bw = bulge * Math.sin(Math.PI * localT);
      const bd = (bot.d > 0 && bot.w > 0) ? bw * (bot.d / bot.w) : bw;
      w += bw; d += bd;
      const so    = bot.so    + (top.so    - bot.so   ) * t;
      const param = bot.param + (top.param - bot.param) * t;
      let prof;
      if (bot.type === top.type) {
        prof = makeProf(w, d, bot.type, param, so);
      } else {
        prof = lerpPro(makeProf(w, d, bot.type, bot.param, bot.so),
                       makeProf(w, d, top.type, top.param, top.so), t);
      }
      return { w, d, so, prof };
    }

    // ── Normal two-segment mode ───────────────────────────────────────────────
    const inLower = z <= p.lBef;
    const t = inLower ? (p.lBef>0?z/p.lBef:0) : (p.lAft>0?(z-p.lBef)/p.lAft:1);
    const A = inLower ? bot : tap, B = inLower ? tap : top;
    const bulge = inLower ? p.bulgeBef : p.bulgeAft;
    let w = A.w+(B.w-A.w)*t, d = A.d+(B.d-A.d)*t;
    const bw = bulge*Math.sin(Math.PI*t);
    const bd = (A.d>0&&A.w>0) ? bw*(A.d/A.w) : bw;
    w+=bw; d+=bd;
    const so    = A.so    + (B.so    - A.so   )*t;
    const param = A.param + (B.param - A.param)*t;
    let prof;
    if (A.type === B.type) {
      prof = makeProf(w, d, A.type, param, so);
    } else {
      prof = lerpPro(makeProf(w,d,A.type,A.param,A.so), makeProf(w,d,B.type,B.param,B.so), t);
    }
    return { w, d, so, prof };
  }

  // Inner hole: always 3-point (guard → mid → pommel).
  // lBef is the midpoint anchor — in no-taper mode the frontend sets this to totalLen/2.
  function innerAt(z) {
    const t = z<=p.lBef ? (p.lBef>0?z/p.lBef:0) : (p.lAft>0?(z-p.lBef)/p.lAft:1);
    if (z<=p.lBef) return [p.hwB+(p.hwM-p.hwB)*t, p.hdB+(p.hdM-p.hdB)*t];
    return [p.hwM+(p.hwT-p.hwM)*t, p.hdM+(p.hdT-p.hdM)*t];
  }

  function maxHD(ow, od, so, iw) {
    const halfY = od/2 - (ow>0 ? so*(iw/2)/(ow/2) : 0);
    return Math.max(0.1, 2*halfY - 2*MIN_WALL);
  }

  const hasHole = p.hwB>0&&p.hdB>0&&p.hwT>0&&p.hdT>0;
  const oProfs=[], iProfs=[], zLvls=[];

  for (let s=0; s<=N_SLICES; s++) {
    const z  = (s/N_SLICES)*totalLen;
    const od = outerAt(z);
    oProfs.push(od.prof);
    if (hasHole) {
      const id   = innerAt(z);
      const maxW = od.w - MIN_WALL*2;
      const maxD = maxHD(od.w, od.d, od.so, id[0]);
      if (id[0]>maxW||id[1]>maxD) clamped=true;
      id[0] = Math.max(0.1, Math.min(id[0], maxW));
      id[1] = Math.max(0.1, Math.min(id[1], maxD));
      iProfs.push(rectProfile(id[0], id[1]));
    }
    zLvls.push(z);
  }

  let tris = [];
  tris = tris.concat(loftSurf(oProfs, zLvls, false));
  if (hasHole) {
    tris = tris.concat(loftSurf(iProfs, zLvls, true));
    tris = tris.concat(ringCap(oProfs[0],        iProfs[0],        0,        false));
    tris = tris.concat(ringCap(oProfs[N_SLICES], iProfs[N_SLICES], totalLen, true));
  } else {
    tris = tris.concat(solidCap(oProfs[0],        0,        false));
    tris = tris.concat(solidCap(oProfs[N_SLICES], totalLen, true));
  }
  return { tris, clamped };
}

// ── Normal helpers ────────────────────────────────────────────────────────────
function vsub(a,b){return[a[0]-b[0],a[1]-b[1],a[2]-b[2]];}
function vcross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function vnorm(v){const l=Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);return l>1e-10?[v[0]/l,v[1]/l,v[2]/l]:[0,0,1];}
function fnorm(t){return vnorm(vcross(vsub(t[1],t[0]),vsub(t[2],t[0])));}

// ── Binary STL serialiser ─────────────────────────────────────────────────────
function toSTL(tris) {
  const buf = Buffer.alloc(84 + tris.length * 50);
  buf.write('GRIP FORGE', 0, 'ascii');
  buf.writeUInt32LE(tris.length, 80);
  let off = 84;
  for (const t of tris) {
    const n = fnorm(t);
    buf.writeFloatLE(n[0], off);   buf.writeFloatLE(n[1], off+4); buf.writeFloatLE(n[2], off+8);
    off += 12;
    for (const v of t) {
      buf.writeFloatLE(v[0], off);   buf.writeFloatLE(v[1], off+4); buf.writeFloatLE(v[2], off+8);
      off += 12;
    }
    buf.writeUInt16LE(0, off); off += 2;
  }
  return buf;
}

// ── Serverless handler ────────────────────────────────────────────────────────
export default function handler(req, res) {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
  if (req.method === 'OPTIONS') { res.status(200).end(); return; }
  if (req.method !== 'POST')   { res.status(405).json({ error: 'POST only' }); return; }

  try {
    const p = req.body;
    const check = (cond, msg) => { if (!cond) throw new Error(msg); };
    check(p.lBef > 0 && p.lAft > 0,  'Length segments must be > 0');
    check(p.wBot > 0 && p.dBot > 0,  'Guard dimensions must be > 0');
    check(p.wTop > 0 && p.dTop > 0,  'Pommel dimensions must be > 0');
    if (!p.noTaper) {
      check(p.wTap > 0 && p.dTap > 0, 'Mid dimensions must be > 0');
    }

    const { tris, clamped } = buildGrip(p);
    const stl = toSTL(tris);

    const totalLen = (p.lBef + p.lAft).toFixed(0);
    res.setHeader('Content-Type',        'application/octet-stream');
    res.setHeader('Content-Disposition', `attachment; filename="grip_${totalLen}mm.stl"`);
    res.setHeader('X-Triangle-Count',    String(tris.length));
    res.setHeader('X-Hole-Clamped',      String(clamped));

    zlib.gzip(stl, (err, compressed) => {
      if (err) { res.status(500).json({ error: 'compression failed' }); return; }
      res.setHeader('Content-Encoding', 'gzip');
      res.setHeader('Content-Length',   compressed.length);
      res.status(200).send(compressed);
    });
  } catch (e) {
    res.status(400).json({ error: e.message });
  }
}
