import { LOS_COLOR, type DemandSettings, type SimStats } from "../lib/sim";

interface Props {
  hasNetwork: boolean;
  running: boolean;
  onRun: (r: boolean) => void;
  speed: number;
  onSpeed: (s: number) => void;
  onReset: () => void;
  demand: DemandSettings;
  onDemand: (d: DemandSettings) => void;
  showTraffic: boolean;
  onShowTraffic: (b: boolean) => void;
  stats: SimStats | null;
  baseline: SimStats | null;
  onSaveBaseline: () => void;
  onClearBaseline: () => void;
  onZoomFocus: (id: string) => void;
}

const SPEEDS = [1, 2, 5, 10, 30];

function fmtTime(t: number) {
  const m = Math.floor(t / 60), s = Math.floor(t % 60);
  return `${m}:${s.toString().padStart(2, "0")}`;
}

/** "-23%" style change, colored so that lower-is-better metrics show green when they drop. */
function Delta({ now, base, lowerIsBetter = true }: { now: number; base: number | undefined; lowerIsBetter?: boolean }) {
  if (base === undefined || base <= 0.5) return null;
  const pct = ((now - base) / base) * 100;
  if (Math.abs(pct) < 1) return <span className="delta">±0%</span>;
  const good = lowerIsBetter ? pct < 0 : pct > 0;
  return <span className={`delta ${good ? "good" : "bad"}`}>{pct > 0 ? "+" : ""}{pct.toFixed(0)}%</span>;
}

function Grade({ los }: { los: string }) {
  return <span className="grade" style={{ background: LOS_COLOR[los] }}>{los}</span>;
}

export default function SimPanel(p: Props) {
  const s = p.stats, b = p.baseline;
  if (!p.hasNetwork) {
    return <p className="hint">Load roads (Build tab) first — then come back here to fill them with traffic.</p>;
  }
  const warm = s && s.t > 300;
  return (
    <>
      <section>
        <div className="row">
          <button className="primary big" onClick={() => p.onRun(!p.running)}>{p.running ? "❚❚ Pause" : "▶ Run"}</button>
          <button onClick={p.onReset}>Reset</button>
          <span className="clock">{s ? fmtTime(s.t) : "0:00"}</span>
        </div>
        <div className="seg">
          {SPEEDS.map((x) => (
            <button key={x} className={p.speed === x ? "on" : ""} onClick={() => p.onSpeed(x)}>{x}×</button>
          ))}
        </div>
        <label>
          Demand (cars / hour)
          <input type="number" min={0} max={60000} step={250} value={p.demand.vehPerHour}
            onChange={(e) => p.onDemand({ ...p.demand, vehPerHour: Number(e.target.value) })} />
        </label>
        <input className="slider" type="range" min={0} max={20000} step={250} value={p.demand.vehPerHour}
          onChange={(e) => p.onDemand({ ...p.demand, vehPerHour: Number(e.target.value) })} />
        <label>
          Local trips
          <input className="slider-sm" type="range" min={0} max={1} step={0.05} value={p.demand.localShare}
            onChange={(e) => p.onDemand({ ...p.demand, localShare: Number(e.target.value) })} />
        </label>
        <p className="hint">
          Most cars enter and leave at the edges of the loaded area; "local trips" start and end inside it. Drivers
          pick the fastest route given current traffic, so they'll find your new roads (and avoid your bad ones).
        </p>
        <label>
          Color roads by traffic
          <input type="checkbox" checked={p.showTraffic} onChange={(e) => p.onShowTraffic(e.target.checked)} />
        </label>
      </section>

      {s && (
        <section>
          <h2>Report card {warm ? "" : "· warming up…"}</h2>
          <div className="scorecard">
            <Grade los={s.los} />
            <div>
              <div className="big-num">{s.nodeDelay.toFixed(1)} s <Delta now={s.nodeDelay} base={b?.nodeDelay} /></div>
              <div className="hint">avg delay per intersection</div>
            </div>
          </div>
          <table className="stats">
            <tbody>
              <tr><td>Extra trip time</td><td>{s.avgDelay.toFixed(0)} s <Delta now={s.avgDelay} base={b?.avgDelay} /></td></tr>
              <tr><td>Trips finished / hr</td><td>{s.throughputPerHour.toFixed(0)} <Delta now={s.throughputPerHour} base={b?.throughputPerHour} lowerIsBetter={false} /></td></tr>
              <tr><td>Avg speed</td><td>{s.avgSpeedMph.toFixed(1)} mph <Delta now={s.avgSpeedMph} base={b?.avgSpeedMph} lowerIsBetter={false} /></td></tr>
              <tr><td>Cars on road</td><td>{s.active}</td></tr>
              <tr><td>Waiting to enter</td><td>{s.pendingSpawns}</td></tr>
              <tr><td>Gave up (gridlock)</td><td className={s.gaveUp > 0 ? "bad" : ""}>{s.gaveUp}</td></tr>
            </tbody>
          </table>
          <div className="row">
            <button onClick={p.onSaveBaseline} disabled={!warm}>Save as "before"</button>
            {b && <button onClick={p.onClearBaseline}>Clear "before"</button>}
          </div>
          <p className="hint">
            Grades follow traffic-engineering Level of Service (A = free flowing … F = failing). Let it run ~5
            sim-minutes, save a "before", make a fix, and watch the % change.
          </p>
        </section>
      )}

      {s && s.focus.length > 0 && (
        <section>
          <h2>Focus zones</h2>
          {s.focus.map((f) => {
            const bf = b?.focus.find((x) => x.id === f.id);
            return (
              <div className="focus-card" key={f.id} onClick={() => p.onZoomFocus(f.id)}>
                <Grade los={f.los} />
                <div className="grow">
                  <strong>{f.name}</strong>
                  <div className="hint">
                    {f.delay.toFixed(1)} s delay <Delta now={f.delay} base={bf?.delay} /> · {f.flow.toFixed(0)} veh/h{" "}
                    <Delta now={f.flow} base={bf?.flow} lowerIsBetter={false} /> · {f.cars} cars · {f.speed.toFixed(0)} mph
                  </div>
                </div>
              </div>
            );
          })}
        </section>
      )}
    </>
  );
}
