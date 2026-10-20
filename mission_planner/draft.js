/* Ordered drawing state. Geometry is never silently clipped or simplified. */
(function (root) {
  'use strict';
  const copy = value => JSON.parse(JSON.stringify(value));
  const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  class RouteDraft {
    constructor({maxPoints = 256, radius = 40} = {}) {
      this.maxPoints = maxPoints; this.radius = radius;
      this.points = [[0, 0]]; this.kinds = ['ordinary']; this.closure = 'open_stop';
      this.past = []; this.future = []; this.gesture = null; this.pending = null;
    }
    snapshot() { return copy({points: this.points, kinds: this.kinds, closure: this.closure}); }
    restore(state) { Object.assign(this, copy(state)); this.gesture = this.pending = null; }
    validate(points, kinds, closure) {
      if (!points.length || distance(points[0], [0, 0]) > 1e-9) throw Error('The route must start at START.');
      for (const p of points) {
        if (p.length !== 2 || !p.every(Number.isFinite)) throw Error('Use valid numbers for each point.');
        if (Math.hypot(...p) > this.radius + 1e-9) throw Error(`Keep the whole line within ${this.radius} m of START. Your previous route is unchanged.`);
      }
      const closingPoint = closure === 'periodic' && distance(points.at(-1), [0, 0]) > 1e-9 ? 1 : 0;
      if (points.length + closingPoint > this.maxPoints) throw Error(`This line has too many points (limit ${this.maxPoints}). Try a shorter stroke or Place points. Your previous route is unchanged.`);
      if (closure === 'periodic' && kinds.slice(1, -1).some(k => k !== 'ordinary')) throw Error('Loops cannot contain marked stops or exact points yet. Use an open route or remove those marks.');
    }
    commit(points, kinds, closure = this.closure) {
      this.validate(points, kinds, closure);
      const next = {points, kinds, closure};
      if (JSON.stringify(next) === JSON.stringify(this.snapshot())) return false;
      this.past.push(this.snapshot()); if (this.past.length > 100) this.past.shift();
      this.future = []; this.restore(next); return true;
    }
    add(point) {
      if (this.pending) throw Error('Connect or discard the waiting line first.');
      if (distance(point, this.points.at(-1)) < 1e-9) return false;
      return this.commit([...this.points, [...point]], [...this.kinds, 'ordinary']);
    }
    begin(point, pointerId, snapRadius) {
      if (this.gesture) return false;
      if (this.pending) throw Error('Connect or discard the waiting line first.');
      const attached = distance(point, this.points.at(-1)) <= snapRadius;
      this.gesture = {pointerId, attached, points: [attached ? [...this.points.at(-1)] : [...point]], error: null};
      return true;
    }
    move(point, pointerId) {
      const g = this.gesture;
      if (!g || g.pointerId !== pointerId || g.error) return;
      if (!point.every(Number.isFinite) || Math.hypot(...point) > this.radius + 1e-9) {
        g.error = `Keep the whole line within ${this.radius} m of START. Your previous route is unchanged.`; return;
      }
      if (distance(point, g.points.at(-1)) > 1e-9) g.points.push([...point]);
      if (g.points.length + this.points.length > this.maxPoints + 1) g.error = `This line has too many points. Try Place points for a longer route. Your previous route is unchanged.`;
    }
    finish(point, pointerId) {
      if (!this.gesture || this.gesture.pointerId !== pointerId) return false;
      this.move(point, pointerId);
      const g = this.gesture; this.gesture = null;
      if (g.error) throw Error(g.error);
      if (g.points.length < 2) return false;
      const additions = g.attached ? g.points.slice(1) : g.points;
      const combined = [...this.points, ...additions];
      const kinds = [...this.kinds, ...additions.map(() => 'ordinary')];
      this.validate(combined, kinds, this.closure);
      if (!g.attached) { this.pending = {points: combined, kinds, closure: this.closure}; return false; }
      return this.commit(combined, kinds);
    }
    connect() { if (!this.pending) return false; const p = this.pending; return this.commit(p.points, p.kinds, p.closure); }
    discard() { this.gesture = this.pending = null; }
    cancel(pointerId) { if (this.gesture?.pointerId === pointerId) this.gesture = null; }
    undo() { if (!this.past.length) return false; this.future.push(this.snapshot()); this.restore(this.past.pop()); return true; }
    redo() { if (!this.future.length) return false; this.past.push(this.snapshot()); this.restore(this.future.pop()); return true; }
    clear() { return this.commit([[0, 0]], ['ordinary'], 'open_stop'); }
    preset(points, closure) { return this.commit(copy(points), points.map(() => 'ordinary'), closure); }
    setClosure(closure) {
      if (!['open_stop', 'periodic'].includes(closure)) throw Error('Choose a supported route ending.');
      return this.commit(this.points, this.kinds, closure);
    }
    setKind(index, kind) {
      if (!Number.isInteger(index) || index <= 0 || index >= this.points.length - 1) throw Error('Mark an interior point. START and END already stop.');
      if (!['ordinary', 'stop', 'exact'].includes(kind)) throw Error('Choose a supported point type.');
      const kinds = [...this.kinds]; kinds[index] = kind;
      return this.commit(this.points, kinds);
    }
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = {RouteDraft};
  else root.PX4Draft = {RouteDraft};
})(typeof globalThis !== 'undefined' ? globalThis : this);
