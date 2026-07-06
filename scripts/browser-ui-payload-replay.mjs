/**
 * 复现 UI 添加球体后的完整加载路径（使用 session store 中的 URDF + mesh）。
 */
import { chromium } from 'playwright';

const BASE_URL = process.env.APP_URL ?? 'http://localhost:5173/';

async function main() {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage();
  await page.goto(BASE_URL, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(
    () =>
      typeof window.__sessionStore !== 'undefined' &&
      window.__sessionStore.getState().simStatus === 'ready',
    null,
    { timeout: 120_000 },
  );

  const replay = await page.evaluate(async () => {
    const state = window.__sessionStore.getState();
    const { appendSpherePayloadWithRecord } = await import('/src/core/payload-editor.ts');
    const { prepareUrdfForMujocoLoad } = await import('/src/utils/urdf-sanitize.ts');
    const { ensureFixedBase } = await import('/src/utils/urdf-base-fixture.ts');
    const { RobotSession } = await import('/src/core/robot-session.ts');

    const { urdfText: appended } = appendSpherePayloadWithRecord(state.urdfText ?? '', {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const fixture = ensureFixedBase(appended, state.baseLink);
    const storeUrdfText = prepareUrdfForMujocoLoad(fixture.urdfText);
    const meshes = state.meshAssets ?? new Map();

    try {
      const session = await RobotSession.create({
        urdfXml: storeUrdfText,
        urdfFileName: state.urdfFileName ?? 'urdf/biped_s70_upper_body.urdf',
        meshes,
        endEffectorLink: state.endEffectorLink,
        baseLink: fixture.baseLink,
        loadPhase: 'payload-reload',
      });
      const n = session.jointNames.length;
      session.dispose();
      return {
        ok: true,
        joints: n,
        meshSize: meshes.size,
        urdfLen: storeUrdfText.length,
        line28: storeUrdfText.split('\n')[27],
      };
    } catch (e) {
      return {
        ok: false,
        error: e?.message ?? String(e),
        meshSize: meshes.size,
        urdfLen: storeUrdfText.length,
        line28: storeUrdfText.split('\n')[27],
      };
    }
  });

  console.log(JSON.stringify(replay, null, 2));
  await browser.close();
  process.exit(replay.ok ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
