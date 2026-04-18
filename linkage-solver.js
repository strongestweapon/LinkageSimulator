/**
 * LinkageSolver — Weighted Levenberg-Marquardt for 2D planar linkages.
 *
 * Uniform model:
 *   - Every body has 3 DOF (x, y, angle). Points stored as local offsets.
 *   - All constraints are "pin equations":
 *       joint:  bodyA.worldPt(a) == bodyB.worldPt(b)       (2 eqs per joint)
 *       pin:    body.worldPt(p)  == (wx, wy)                (2 eqs per pin)
 *   - Each constraint has a weight. High weight = strong preference.
 *   - `setFixed(body)` removes a body from variables (for ground frame bodies
 *     whose entire pose is known).
 *
 * Typical weighting convention:
 *   ground pins : 100
 *   input pins  : 100
 *   joints      : 1
 * LM minimizes Σ (w·residual)², so weighted residuals drive the solution.
 */
class LinkageSolver {
  constructor() {
    this.bodies = {};    // name → { x, y, angle, locals: {name: {lx, ly}} }
    this.joints = [];    // [{ bodyA, ptA, bodyB, ptB, weight }]
    this.pins = [];      // [{ body, pt, wx, wy, weight }]
    this.fixed = {};     // name → true (body removed from variables)
    this.lastResidual = 0;
  }

  clear() {
    this.bodies = {}; this.joints = []; this.pins = []; this.fixed = {};
  }

  addBody(name, points) {
    if (points.length < 2) return;
    let cx = 0, cy = 0;
    for (const p of points) { cx += p.x; cy += p.y; }
    cx /= points.length; cy /= points.length;
    const locals = {};
    for (const p of points) locals[p.name] = { lx: p.x - cx, ly: p.y - cy };
    this.bodies[name] = { x: cx, y: cy, angle: 0, locals };
  }

  addJoint(bodyA, ptA, bodyB, ptB, weight) {
    this.joints.push({ bodyA, ptA, bodyB, ptB, weight: weight || 1 });
  }

  setFixed(bodyName) { this.fixed[bodyName] = true; }

  clearPins() { this.pins = []; }

  addPin(body, pt, worldPos, weight) {
    this.pins.push({ body, pt, wx: worldPos.x, wy: worldPos.y, weight: weight || 1 });
  }

  getWorldPoint(bodyName, ptName) {
    const b = this.bodies[bodyName];
    if (!b) return null;
    const lp = b.locals[ptName];
    if (!lp) return null;
    const c = Math.cos(b.angle), s = Math.sin(b.angle);
    return { x: b.x + lp.lx * c - lp.ly * s, y: b.y + lp.lx * s + lp.ly * c };
  }

  getWorldPoints(bodyName) {
    const b = this.bodies[bodyName];
    if (!b) return {};
    const c = Math.cos(b.angle), s = Math.sin(b.angle);
    const out = {};
    for (const name in b.locals) {
      const lp = b.locals[name];
      out[name] = { x: b.x + lp.lx * c - lp.ly * s, y: b.y + lp.lx * s + lp.ly * c };
    }
    return out;
  }

  solve(maxIter, tolerance) {
    maxIter = maxIter || 100;
    tolerance = tolerance || 1e-8;

    // Variable layout: 3 DOF per non-fixed body
    const varNames = [];
    for (const name in this.bodies) if (!this.fixed[name]) varNames.push(name);
    const n = varNames.length * 3;
    if (n === 0) return true;

    const m = this.joints.length * 2 + this.pins.length * 2;
    if (m === 0) return true;

    const self = this;

    const packState = () => {
      const q = new Float64Array(n);
      for (let i = 0; i < varNames.length; i++) {
        const b = self.bodies[varNames[i]];
        q[i*3] = b.x; q[i*3+1] = b.y; q[i*3+2] = b.angle;
      }
      return q;
    };

    const unpackState = (q) => {
      for (let i = 0; i < varNames.length; i++) {
        const b = self.bodies[varNames[i]];
        b.x = q[i*3]; b.y = q[i*3+1]; b.angle = q[i*3+2];
      }
    };

    const evalF = () => {
      const F = new Float64Array(m);
      let idx = 0;
      for (const j of self.joints) {
        const wa = self.getWorldPoint(j.bodyA, j.ptA);
        const wb = self.getWorldPoint(j.bodyB, j.ptB);
        const w = j.weight;
        if (!wa || !wb) { idx += 2; continue; }
        F[idx++] = (wa.x - wb.x) * w;
        F[idx++] = (wa.y - wb.y) * w;
      }
      for (const p of self.pins) {
        const wp = self.getWorldPoint(p.body, p.pt);
        const w = p.weight;
        if (!wp) { idx += 2; continue; }
        F[idx++] = (wp.x - p.wx) * w;
        F[idx++] = (wp.y - p.wy) * w;
      }
      return F;
    };

    const eps = 1e-6;
    const computeJ = (q) => {
      const J = [];
      for (let i = 0; i < m; i++) J.push(new Float64Array(n));
      const F0 = evalF();
      for (let j = 0; j < n; j++) {
        const old = q[j];
        q[j] = old + eps;
        unpackState(q);
        const F1 = evalF();
        for (let i = 0; i < m; i++) J[i][j] = (F1[i] - F0[i]) / eps;
        q[j] = old;
      }
      unpackState(q);
      return J;
    };

    const solveLM = (J, F, lambda) => {
      // Build JᵀJ and JᵀF
      const A = [];
      const diag = new Float64Array(n);
      for (let i = 0; i < n; i++) {
        A.push(new Float64Array(n));
        for (let j = 0; j < n; j++) {
          let s = 0;
          for (let k = 0; k < m; k++) s += J[k][i] * J[k][j];
          A[i][j] = s;
        }
        diag[i] = A[i][i];
      }
      for (let i = 0; i < n; i++) A[i][i] += lambda * Math.max(diag[i], 1e-6);
      const b = new Float64Array(n);
      for (let i = 0; i < n; i++) {
        let s = 0;
        for (let k = 0; k < m; k++) s += J[k][i] * F[k];
        b[i] = -s;
      }
      // Gaussian elimination with partial pivoting
      for (let col = 0; col < n; col++) {
        let maxVal = Math.abs(A[col][col]), maxRow = col;
        for (let row = col + 1; row < n; row++) {
          if (Math.abs(A[row][col]) > maxVal) { maxVal = Math.abs(A[row][col]); maxRow = row; }
        }
        if (maxVal < 1e-14) continue;
        if (maxRow !== col) { [A[col], A[maxRow]] = [A[maxRow], A[col]]; [b[col], b[maxRow]] = [b[maxRow], b[col]]; }
        for (let row = col + 1; row < n; row++) {
          const f = A[row][col] / A[col][col];
          for (let j = col; j < n; j++) A[row][j] -= f * A[col][j];
          b[row] -= f * b[col];
        }
      }
      const dq = new Float64Array(n);
      for (let i = n - 1; i >= 0; i--) {
        let s = b[i];
        for (let j = i + 1; j < n; j++) s -= A[i][j] * dq[j];
        dq[i] = Math.abs(A[i][i]) > 1e-14 ? s / A[i][i] : 0;
      }
      return dq;
    };

    let lambda = 1e-3;
    let q = packState();
    let finalNorm2 = Infinity;

    for (let iter = 0; iter < maxIter; iter++) {
      unpackState(q);
      const F = evalF();
      let norm2 = 0;
      for (let i = 0; i < m; i++) norm2 += F[i] * F[i];
      finalNorm2 = norm2;
      if (norm2 < tolerance * tolerance) { this.lastResidual = Math.sqrt(norm2); return true; }

      const J = computeJ(q);
      const dq = solveLM(J, F, lambda);

      const q2 = new Float64Array(n);
      for (let i = 0; i < n; i++) q2[i] = q[i] + dq[i];
      unpackState(q2);
      const F2 = evalF();
      let norm2_new = 0;
      for (let i = 0; i < m; i++) norm2_new += F2[i] * F2[i];

      if (norm2_new < norm2) {
        q = q2;
        lambda = Math.max(lambda * 0.3, 1e-12);
        finalNorm2 = norm2_new;
      } else {
        unpackState(q);
        lambda = Math.min(lambda * 5, 1e8);
      }
    }
    unpackState(q);
    this.lastResidual = Math.sqrt(finalNorm2);
    return false;
  }
}

if (typeof module !== 'undefined' && module.exports) module.exports = LinkageSolver;
