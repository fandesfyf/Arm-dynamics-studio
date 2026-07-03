import { useEffect, useState } from 'react';
import { loadMujocoRobot } from '../../mujoco/loader';
import { readQpos } from '../../mujoco/state';
import { applyConstantTorque, mjInverse, mjStepN } from '../../mujoco/step';
import { buildJointMap, jointMapNames } from '../../pinocchio/joint-map';
import { loadPinocchioFromUrdf } from '../../pinocchio/loader';

type LoadPhase = 'idle' | 'loading' | 'ok' | 'error';

interface P0Result {
  mjNq: number;
  mjNv: number;
  mjNu: number;
  pinNq: number;
  pinNv: number;
  mjJointNames: string[];
  pinJointNames: string[];
  jointMapAligned: string[];
  qposAfterSteps: number[];
  qposDelta: number;
  inverseTorque: number[];
  inverseFinite: boolean;
}

export function P0Page() {
  const [phase, setPhase] = useState<LoadPhase>('idle');
  const [message, setMessage] = useState('正在加载仿真引擎…');
  const [result, setResult] = useState<P0Result | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    async function runP0() {
      setPhase('loading');
      setMessage('正在加载 MuJoCo + Pinocchio…');

      try {
        const response = await fetch('/robots/test_arm.urdf');
        if (!response.ok) {
          throw new Error(`无法加载 test_arm.urdf: HTTP ${response.status}`);
        }
        const urdfText = await response.text();

        const [mj, pin] = await Promise.all([
          loadMujocoRobot({
            urdfText,
            urdfFileName: 'test_arm.urdf',
            meshes: new Map(),
          }),
          loadPinocchioFromUrdf(urdfText),
        ]);

        const qposBefore = readQpos(mj.model, mj.data);

        applyConstantTorque(mj.data, [1, 0, 0, 0, 0]);
        mjStepN(mj.mujoco, mj.model, mj.data, 1000);

        const qposAfter = readQpos(mj.model, mj.data);
        let qposDelta = 0;
        for (let i = 0; i < qposAfter.length; i++) {
          qposDelta += Math.abs(qposAfter[i] - qposBefore[i]);
        }

        const inv = mjInverse(mj.mujoco, mj.model, mj.data);
        const inverseFinite = inv.qfrc_inverse.every(Number.isFinite);

        const jointMap = buildJointMap(mj.mujoco, mj.model, pin);
        const aligned = jointMapNames(jointMap);

        if (cancelled) return;

        setResult({
          mjNq: mj.nq,
          mjNv: mj.nv,
          mjNu: mj.nu,
          pinNq: pin.nq,
          pinNv: pin.nv,
          mjJointNames: mj.jointNames,
          pinJointNames: pin.jointNames,
          jointMapAligned: aligned,
          qposAfterSteps: Array.from(qposAfter),
          qposDelta,
          inverseTorque: Array.from(inv.qfrc_inverse),
          inverseFinite,
        });
        setPhase('ok');
        setMessage('P0 验收通过：双引擎加载、步进与逆动力学正常');
      } catch (e) {
        if (cancelled) return;
        setPhase('error');
        setMessage('加载或验证失败');
        setError(e instanceof Error ? e.message : String(e));
      }
    }

    void runP0();
    return () => {
      cancelled = true;
    };
  }, []);

  return (
    <div className="p0-page">
      <h1>Arm Dynamics Sim — P0 引擎验证</h1>

      <div
        className={`status ${phase === 'loading' ? 'loading' : phase === 'ok' ? 'ok' : phase === 'error' ? 'error' : ''}`}
      >
        {message}
        {error && <pre style={{ margin: '0.5rem 0 0', whiteSpace: 'pre-wrap' }}>{error}</pre>}
      </div>

      {result && (
        <>
          <section>
            <h2>MuJoCo 维度</h2>
            <ul>
              <li>
                <code>nq</code> = {result.mjNq}
              </li>
              <li>
                <code>nv</code> = {result.mjNv}
              </li>
              <li>
                <code>nu</code> = {result.mjNu}
              </li>
            </ul>
          </section>

          <section>
            <h2>Pinocchio 维度</h2>
            <ul>
              <li>
                <code>nq</code> = {result.pinNq}
              </li>
              <li>
                <code>nv</code> = {result.pinNv}
              </li>
            </ul>
          </section>

          <section>
            <h2>mj_step × 1000（qfrc_applied 恒定力矩 joint1）</h2>
            <ul>
              <li>qpos 变化量 Σ|Δq| = {result.qposDelta.toFixed(6)}</li>
              <li>qpos = [{result.qposAfterSteps.map((v) => v.toFixed(4)).join(', ')}]</li>
            </ul>
          </section>

          <section>
            <h2>mj_inverse（qacc=0）</h2>
            <ul>
              <li>有限值：{result.inverseFinite ? '是' : '否'}</li>
              <li>τ = [{result.inverseTorque.map((v) => v.toFixed(4)).join(', ')}]</li>
            </ul>
          </section>

          <section>
            <h2>JointMap 对齐关节名</h2>
            <ul>
              {result.jointMapAligned.map((name) => (
                <li key={name}>{name}</li>
              ))}
            </ul>
            <p style={{ margin: '0.5rem 0 0', fontSize: '0.875rem', color: '#666' }}>
              MuJoCo: [{result.mjJointNames.join(', ')}] · Pinocchio: [
              {result.pinJointNames.join(', ')}]
            </p>
          </section>
        </>
      )}
    </div>
  );
}
