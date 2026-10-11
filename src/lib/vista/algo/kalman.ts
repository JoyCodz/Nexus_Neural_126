/** Constant-velocity Kalman filter on the ground plane. State: [x, y, vx, vy]. */
export class PlaneKalman {
  x = 0;
  y = 0;
  vx = 0;
  vy = 0;
  p00 = 4;
  p11 = 4;
  p22 = 2;
  p33 = 2;

  reset(x: number, y: number): void {
    this.x = x;
    this.y = y;
    this.vx = 0;
    this.vy = 0;
    this.p00 = 2;
    this.p11 = 2;
    this.p22 = 1.5;
    this.p33 = 1.5;
  }

  predict(dt: number, q = 0.35): void {
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    const dt2 = dt * dt;
    this.p00 += 2 * this.p22 * dt + q * dt2;
    this.p11 += 2 * this.p33 * dt + q * dt2;
    this.p22 += q * dt;
    this.p33 += q * dt;
  }

  update(zx: number, zy: number, r = 0.25): void {
    const k0 = this.p00 / (this.p00 + r);
    const k1 = this.p11 / (this.p11 + r);
    const ix = zx - this.x;
    const iy = zy - this.y;
    this.x += k0 * ix;
    this.y += k1 * iy;
    this.vx += k0 * ix * 0.45;
    this.vy += k1 * iy * 0.45;
    this.p00 *= 1 - k0;
    this.p11 *= 1 - k1;
  }

  predictAt(horizon: number): { x: number; y: number } {
    return { x: this.x + this.vx * horizon, y: this.y + this.vy * horizon };
  }
}
