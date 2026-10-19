// Shared Bluetooth code for the Foot Pod pages: connects to FootPod-L / FootPod-R
// running footpod_capture.ino and decodes its raw IMU packets.
const FootPod = (() => {
  const SVC = 'a3c40000-5f1d-4b8e-9c3a-7d1e2f3b4c50';
  const CHR = 'a3c40001-5f1d-4b8e-9c3a-7d1e2f3b4c50';
  const STAT = 'a3c40002-5f1d-4b8e-9c3a-7d1e2f3b4c50';   // sleep status, firmware v2+
  const PKT_LEN = 236;                       // 8-byte header + 19 samples x 12 bytes
  const ACC = 0.000488, GYR = 0.070;         // g/LSB (+-16 g), dps/LSB (+-2000 dps)
  const ACC_CLIP = 15.9, GYR_CLIP = 1990;

  const supported = !!navigator.bluetooth;
  const sideFromName = name => /-?R$/i.test(name || '') ? 'R' : 'L';

  async function requestDevice() {
    return navigator.bluetooth.requestDevice({
      filters: [{ services: [SVC] }, { namePrefix: 'FootPod' }],
      optionalServices: [SVC]
    });
  }

  // handlers: onSample(pod, tSec, [ax,ay,az] g, [gx,gy,gz] dps), onPacket(pod, bytes, rxMs, lost),
  //           onStatus(pod, text, cls), onChange(pod), onMtu(pod)
  class Pod {
    constructor(side, handlers) {
      this.side = side;
      this.h = handlers || {};
      this.device = null; this.chr = null;
      this.wantConnected = false; this.retryTimer = null;
      this.fs = 416; this.lastT0 = null; this.wrap = 0; this.lastSeq = null;
      this.lostTotal = 0; this.badLen = 0; this.clipCount = 0; this.rateCount = 0; this.lastRx = 0;
      this.stat = null;   // {uptime, still, limit, usb, woke, connected} from the pod, if its firmware sends it
    }
    get connected() { return !!(this.device && this.device.gatt.connected); }
    get name() { return (this.device && this.device.name) || ('FootPod-' + this.side); }
    get live() { return this.connected && performance.now() - this.lastRx < 1500; }
    status(text, cls) { this.h.onStatus && this.h.onStatus(this, text, cls); }
    changed() { this.h.onChange && this.h.onChange(this); }

    async attach(device) {
      if (this.device && this.device !== device && this.connected) this.device.gatt.disconnect();
      if (this.device !== device) {
        this.device = device;
        device.addEventListener('gattserverdisconnected', () => this._onDisconnected());
      }
      this.wantConnected = true;
      await this._connectGatt();
    }

    async _connectGatt() {
      this.status('Connecting…', 'retry'); this.changed();
      const server = await this.device.gatt.connect();
      const svc = await server.getPrimaryService(SVC);
      this.chr = await svc.getCharacteristic(CHR);
      if (!this.chr._fpBound) {
        this.chr.addEventListener('characteristicvaluechanged', e => this._onNotify(e));
        this.chr._fpBound = true;
      }
      await this.chr.startNotifications();
      try {
        const sc = await svc.getCharacteristic(STAT);
        if (!sc._fpBound) { sc.addEventListener('characteristicvaluechanged', e => this._onStat(e.target.value)); sc._fpBound = true; }
        await sc.startNotifications();
      } catch (e) { this.stat = null; }   // older firmware: no status
      this.lastSeq = null;
      this.status(this.name, 'on'); this.changed();
    }

    disconnect() {
      this.wantConnected = false;
      clearTimeout(this.retryTimer);
      if (this.connected) this.device.gatt.disconnect();
      this.status('Not connected'); this.changed();
    }

    _onDisconnected() {
      this.changed();
      if (!this.wantConnected) { this.status('Not connected'); return; }
      this.status('Reconnecting… (asleep? tap it)', 'retry');
      const attempt = async () => {
        if (!this.wantConnected) return;
        try { await this._connectGatt(); }
        catch (e) { this.retryTimer = setTimeout(attempt, 2000); }
      };
      this.retryTimer = setTimeout(attempt, 1000);
    }

    _onStat(dv) {
      if (dv.byteLength < 10) return;
      const f = dv.getUint8(8);
      this.stat = { uptime: dv.getUint32(0, true), still: dv.getUint16(4, true), limit: dv.getUint16(6, true),
                    usb: !!(f & 1), woke: !!(f & 2), connected: !!(f & 4) };
      this.h.onStat && this.h.onStat(this);
      if (this._diag) this._diag(this);
    }

    _onNotify(e) {
      const dv = e.target.value;
      if (dv.byteLength !== PKT_LEN) { this.badLen++; this.h.onMtu && this.h.onMtu(this); return; }
      const rx = Date.now();
      this.lastRx = performance.now();
      const t0 = dv.getUint32(0, true), seq = dv.getUint16(4, true), n = dv.getUint8(6), od = dv.getUint8(7);
      if (this.lastT0 !== null && t0 < this.lastT0 - 2 ** 31) this.wrap += 2 ** 32;   // micros() rollover
      this.lastT0 = t0;
      let lost = 0;
      if (this.lastSeq !== null) { lost = (seq - this.lastSeq - 1) & 0xFFFF; this.lostTotal += lost; }
      this.lastSeq = seq;
      this.fs = od * 8;
      const dt = 1e6 / this.fs;
      if (this.h.onPacket) {
        this.h.onPacket(this, new Uint8Array(dv.buffer.slice(dv.byteOffset, dv.byteOffset + dv.byteLength)), rx, lost);
      }
      for (let i = 0; i < n; i++) {
        const o = 8 + i * 12;
        const g = [dv.getInt16(o, true) * GYR, dv.getInt16(o + 2, true) * GYR, dv.getInt16(o + 4, true) * GYR];
        const a = [dv.getInt16(o + 6, true) * ACC, dv.getInt16(o + 8, true) * ACC, dv.getInt16(o + 10, true) * ACC];
        if (Math.abs(a[0]) >= ACC_CLIP || Math.abs(a[1]) >= ACC_CLIP || Math.abs(a[2]) >= ACC_CLIP ||
            Math.abs(g[0]) >= GYR_CLIP || Math.abs(g[1]) >= GYR_CLIP || Math.abs(g[2]) >= GYR_CLIP) this.clipCount++;
        this.h.onSample && this.h.onSample(this, (t0 + this.wrap + i * dt) / 1e6, a, g);
      }
      this.rateCount += n;
    }
  }

  // Builds the Left / Right connection chips into `container`, with one Connect button.
  function connectionUI(container, pods, opts = {}) {
    container.innerHTML = `
      <button class="primary fp-connect" style="width:100%">Connect a pod</button>
      <div class="pods"></div>
      <p class="note">${opts.note || 'Tap Connect once per pod. Each pod is placed on its side (L or R) from its name.'}</p>`;
    const btn = container.querySelector('.fp-connect');
    const chips = {};
    for (const side of ['L', 'R']) {
      const el = document.createElement('div');
      el.className = 'podchip';
      el.innerHTML = `<div class="top"><span class="swatch" style="background:var(--pod-${side})"></span>${side === 'L' ? 'Left' : 'Right'}</div>
        <div class="st"><span class="dot"></span><span class="tx">Not connected</span></div>
        <div class="diag"></div>
        <button class="small disc" disabled>Disconnect</button>`;
      container.querySelector('.pods').appendChild(el);
      chips[side] = el;
      el.querySelector('.disc').onclick = () => pods[side].disconnect();
      const fmt = s => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
      pods[side]._diag = p => {
        const st = p.stat, d = el.querySelector('.diag');
        if (!st || !p.connected) { d.textContent = ''; return; }
        d.textContent = st.usb ? `On USB, won't sleep · still ${fmt(st.still)}`
          : `Still ${fmt(st.still)} of ${fmt(st.limit)} before sleep` + (st.woke ? ` · woke from sleep ${fmt(st.uptime)} ago` : '');
      };
    }
    const refresh = () => {
      for (const side of ['L', 'R']) {
        const p = pods[side];
        if (!p.connected) chips[side].querySelector('.diag').textContent = '';
        chips[side].querySelector('.disc').disabled = !p.connected && !p.wantConnected;
      }
      btn.disabled = !supported || (pods.L.connected && pods.R.connected);
    };
    const setStatus = (p, text, cls) => {
      chips[p.side].querySelector('.tx').textContent = text;
      chips[p.side].querySelector('.dot').className = 'dot' + (cls ? ' ' + cls : '');
    };
    btn.onclick = async () => {
      try {
        const device = await requestDevice();
        const p = pods[sideFromName(device.name)];
        if (opts.beforeAttach) opts.beforeAttach(p, device);
        await p.attach(device);
      } catch (e) {
        if (e.name !== 'NotFoundError') alert('Connect failed: ' + e.message);
      }
    };
    refresh();
    return { refresh, setStatus };
  }

  return { Pod, requestDevice, sideFromName, connectionUI, supported, SVC, CHR, STAT, PKT_LEN, ACC, GYR };
})();
