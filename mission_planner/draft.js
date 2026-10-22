/* Ordered drawing state; the studio can opt into disclosed best-effort capture. */
(function (root) {
  'use strict';
  const copy = value => JSON.parse(JSON.stringify(value));
  const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  class RouteDraft {
    constructor({maxPoints = 256, radius = 40, bestEffort = false} = {}) {
      this.maxPoints = maxPoints; this.radius = radius; this.bestEffort = bestEffort;
      this.samplingNotice = '';
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
      if (!this.bestEffort && closure === 'periodic' && kinds.slice(1, -1).some(k => k !== 'ordinary')) throw Error('Loops cannot contain marked stops or exact points yet. Use an open route or remove those marks.');
    }
    commit(points, kinds, closure = this.closure) {
      if (this.bestEffort) {
        const limit = this.maxPoints - (closure === 'periodic' ? 1 : 0);
        while (points.length > limit) {
          const indices = points.map((_, i) => i).filter(i => i === 0 || i === points.length-1 || i % 2 === 0);
          points = indices.map(i => points[i]); kinds = indices.map(i => kinds[i]);
          this.samplingNotice = 'Dense mouse samples were thinned in order to keep drawing responsive. Review the displayed drawing before planning.';
        }
      }
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
      if (this.bestEffort && g.points.length > this.maxPoints) {
        g.points = g.points.filter((_, i, a) => i === 0 || i === a.length-1 || i % 2 === 0);
        this.samplingNotice = 'Dense mouse samples were thinned in order to keep drawing responsive. Review the displayed drawing before planning.';
      } else if (!this.bestEffort && g.points.length + this.points.length > this.maxPoints + 1) g.error = `This line has too many points. Try Place points for a longer route. Your previous route is unchanged.`;
    }
    finish(point, pointerId) {
      if (!this.gesture || this.gesture.pointerId !== pointerId) return false;
      this.move(point, pointerId);
      const g = this.gesture; this.gesture = null;
      if (g.error) throw Error(g.error);
      if (g.points.length < 2) return this.bestEffort ? this.add(g.points[0]) : false;
      const additions = g.attached ? g.points.slice(1) : g.points;
      const combined = [...this.points, ...additions];
      const kinds = [...this.kinds, ...additions.map(() => 'ordinary')];
      if (this.bestEffort) return this.commit(combined, kinds);
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
