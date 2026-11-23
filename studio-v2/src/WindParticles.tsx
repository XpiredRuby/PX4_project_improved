import { useEffect, useRef } from 'react';

/** Animated wind streaks over the map. Direction is meteorological (wind FROM, degrees). */
export default function WindParticles({ speed, directionFrom }: { speed: number; directionFrom: number }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas || !(speed > 0.2)) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const reduce = window.matchMedia?.('(prefers-reduced-motion: reduce)').matches;
    const to = (directionFrom + 180) * Math.PI / 180;
    const ux = Math.sin(to), uy = -Math.cos(to); // screen: east right, north up
    const pxPerS = 10 + speed * 9;
    let w = 0, h = 0, raf = 0, last = performance.now();
    const resize = () => {
      const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
      w = r.width; h = r.height; canvas.width = w * dpr; canvas.height = h * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    };
    resize();
    const ro = new ResizeObserver(resize); ro.observe(canvas);
    const count = Math.round(Math.min(220, 40 + speed * 18));
    const ps = Array.from({ length: count }, () => ({ x: Math.random() * w, y: Math.random() * h, age: Math.random() * 3 }));
    const draw = (now: number) => {
      const dt = Math.min(0.05, (now - last) / 1000); last = now;
      ctx.clearRect(0, 0, w, h);
      ctx.lineCap = 'round';
      for (const p of ps) {
        p.x += ux * pxPerS * dt; p.y += uy * pxPerS * dt; p.age += dt;
        if (p.x < -20 || p.x > w + 20 || p.y < -20 || p.y > h + 20 || p.age > 4) { p.x = Math.random() * w; p.y = Math.random() * h; p.age = 0; }
        const a = Math.min(1, p.age) * Math.max(0, 1 - (p.age - 3)) * 0.45;
        ctx.strokeStyle = `rgba(200, 225, 240, ${a})`;
        ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p.x - ux * (6 + speed * 1.5), p.y - uy * (6 + speed * 1.5)); ctx.stroke();
      }
      if (!reduce) raf = requestAnimationFrame(draw);
    };
    draw(last);
    return () => { cancelAnimationFrame(raf); ro.disconnect(); };
  }, [speed, directionFrom]);
  return <canvas ref={ref} className="wind-particles" aria-hidden="true"/>;
}
