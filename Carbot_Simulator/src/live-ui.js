import {dashboardOdom, alignAtStart, coursePose} from './live-car.js';

const element = id => document.getElementById(id);

export class LiveCarUI {
  constructor(getSimulation) {
    this.getSimulation = getSimulation;
    this.connected = false;
    this.raw = null;
    this.lastRaw = null;
    this.lastSuccessMs = 0;
    this.alignment = null;
    this.pose = null;
    this.trail = [];
    this.timer = null;
    this.pending = false;
    this.world = null;
    this.status = 'Start live_server.py on this laptop, then connect.';
  }

  install() {
    element('live-connect').onclick = () => this.connect();
    element('live-align').onclick = () => this.align();
    element('live-disconnect').onclick = () => this.disconnect();
    this.updateControls();
  }

  connect() {
    if (this.connected) return;
    this.connected = true;
    const sim = this.getSimulation();
    if (sim) sim.running = false;
    this.status = 'Connecting to Risabot 1 dashboard…';
    this.updateControls();
    this.poll();
    this.timer = setInterval(() => this.poll(), 200);
  }

  align() {
    const sim = this.getSimulation();
    if (!this.connected || !this.raw || performance.now() - this.lastSuccessMs > 700 || !sim) return;
    this.alignment = alignAtStart(this.raw, sim.course.start);
    this.trail = [];
    this.pose = coursePose(this.raw, this.alignment);
    sim.world.updateLiveCar(this.pose);
    this.status = 'Aligned at START · real car odometry now drives the orange model.';
    this.updateControls();
  }

  disconnect() {
    this.connected = false;
    clearInterval(this.timer);
    this.timer = null;
    this.raw = this.lastRaw = this.alignment = this.pose = null;
    this.lastSuccessMs = 0;
    this.trail = [];
    this.getSimulation()?.world.updateLiveCar(null);
    this.status = 'Disconnected · simulation available.';
    this.updateControls();
  }

  async poll() {
    if (!this.connected || this.pending) return;
    this.pending = true;
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(), 1500);
    try {
      const response = await fetch('/api/live', {cache: 'no-store', signal: abort.signal});
      if (!response.ok) throw new Error('Dashboard did not respond');
      const raw = dashboardOdom(await response.json());
      if (!raw) throw new Error('hardware /odom is stale or unavailable');
      if (!this.connected) return;
      if (this.lastRaw && Math.hypot(raw.x - this.lastRaw.x, raw.y - this.lastRaw.y) > 0.35) {
        this.alignment = null;
        this.trail = [];
        this.status = 'Odometry jumped; return car to START and align again.';
      }
      this.raw = raw;
      this.lastRaw = raw;
      this.lastSuccessMs = performance.now();
      if (this.alignment) {
        this.pose = coursePose(raw, this.alignment);
        const last = this.trail.at(-1);
        if (!last || Math.hypot(this.pose.x - last.x, this.pose.y - last.y) > 0.005) {
          this.trail.push({...this.pose});
          if (this.trail.length > 3000) this.trail.shift();
        }
        this.getSimulation()?.world.updateLiveCar(this.pose);
        this.status = `LIVE · odom ${raw.age.toFixed(2)} s old · course (${this.pose.x.toFixed(2)}, ${this.pose.y.toFixed(2)}) m · heading ${(this.pose.a * 180 / Math.PI).toFixed(1)}°`;
      } else {
        this.pose = null;
        this.getSimulation()?.world.updateLiveCar(null);
        if (!this.status.startsWith('Odometry jumped')) this.status = 'Connected · place Risabot 1 at START, then click Align at START.';
      }
    } catch (error) {
      if (!this.connected) return;
      this.raw = this.pose = null;
      this.getSimulation()?.world.updateLiveCar(null);
      this.status = `Waiting for real car: ${error.message}. Run live_server.py if this page was opened as a file.`;
    } finally {
      clearTimeout(timeout);
      this.pending = false;
      this.updateControls();
    }
  }

  updateControls() {
    element('live-connect').disabled = this.connected;
    element('live-align').disabled = !this.connected || !this.raw || performance.now() - this.lastSuccessMs > 700;
    element('live-disconnect').disabled = !this.connected;
    element('live-status').textContent = this.status;
    document.body.classList.toggle('live-active', this.connected);
    element('run').disabled = this.connected || !this.getSimulation();
    element('step').disabled = this.connected;
    element('csv').disabled = this.connected;
    element('estop').disabled = this.connected;
    element('estop').textContent = this.connected ? 'Simulation E-stop disabled' : this.getSimulation()?.estop ? 'Release stop' : 'Emergency stop';
  }

  tick() {
    const sim = this.getSimulation();
    if (!sim) return;
    if (this.world !== sim.world) {
      this.world = sim.world;
      this.raw = this.lastRaw = this.alignment = this.pose = null;
      this.lastSuccessMs = 0;
      this.trail = [];
      if (this.connected) this.status = 'Course reset · place car at START and align again.';
      sim.world.updateLiveCar(null);
    }
    sim.world.car.visible = !this.connected;
    if (!this.connected) return;
    if (this.pose && performance.now() - this.lastSuccessMs > 700) {
      this.raw = this.pose = null;
      sim.world.updateLiveCar(null);
      this.status = 'Hardware odometry timed out; waiting for fresh data.';
    }
    sim.running = false;
    element('state').textContent = this.pose ? 'LIVE CAR · ODOM' : 'LIVE CAR · WAITING';
    element('reason').textContent = 'Read-only view of hardware odometry; physical position and pose error are not measured here.';
    element('mission').textContent = 'RISABOT 1';
    element('clock').textContent = this.raw ? `${this.raw.age.toFixed(2)} s age` : '—';
    element('speed-value').textContent = this.raw ? `${(this.raw.speed * 100).toFixed(1)} cm/s` : '—';
    element('error-value').textContent = 'N/A';
    element('winner').textContent = 'NO COMMAND';
    element('guidance-mode').textContent = 'Hardware odometry display';
    element('confidence-value').textContent = 'Unmeasured';
    element('uwb-weight').textContent = 'Not used in this view';
    element('branch-value').textContent = this.alignment ? 'START aligned' : 'Align at START';
    element('run').textContent = 'Simulation paused';
    for (const which of ['a', 'b']) {
      const select = element(`select-${which}`), caption = element(`caption-${which}`);
      if (select?.value === 'world') caption.textContent = 'Orange model = real Risabot 1 odometry estimate · drag to orbit · Follow car tracks it';
      else if (select?.value === 'map') caption.textContent = 'Orange outline and trail = real car odometry estimate · start alignment required';
      else if (caption) caption.textContent = 'Simulated view is paused while live odometry is connected.';
    }
    this.updateControls();
  }
}
