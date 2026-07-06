/**
 * 浏览器 WASM MuJoCo 负载回归：需 dev server 已启动 (npm run dev)。
 * 用法: node scripts/browser-wasm-payload-test.mjs
 */
import { chromium } from 'playwright';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BASE_URL = process.env.APP_URL ?? 'http://localhost:5173/';
const DUMP_PATH = join(
  __dirname,
  '../debug-dumps/mujoco-failed-2026-07-04T05-16-29-383Z/urdf/biped_s70_upper_body.urdf',
);

async function main() {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage();
  const consoleLogs = [];
  const pageErrors = [];
  page.on('console', (msg) => consoleLogs.push(`[${msg.type()}] ${msg.text()}`));
  page.on('pageerror', (err) => pageErrors.push(String(err)));

  await page.goto(BASE_URL, { waitUntil: 'domcontentloaded', timeout: 120_000 });
  await page.waitForTimeout(3000);

  const dump = readFileSync(DUMP_PATH, 'utf-8');

  const wasmResults = await page.evaluate(async (urdf) => {
    const { loadMujocoRobot } = await import('/src/mujoco/loader.ts');
    const { prepareUrdfForMujocoLoad } = await import('/src/utils/urdf-sanitize.ts');
    const { appendSpherePayloadWithRecord } = await import('/src/core/payload-editor.ts');

    const run = async (label, text, phase) => {
      try {
        const { releaseActiveMujocoHandles } = await import('/src/mujoco/loader.ts');
        releaseActiveMujocoHandles();
        const fixed = prepareUrdfForMujocoLoad(text);
        const bundle = await loadMujocoRobot({
          urdfText: fixed,
          urdfFileName: 'urdf/biped_s70_upper_body.urdf',
          meshes: new Map(),
          loadPhase: phase,
          urdfPrepared: true,
        });
        const joints = bundle.jointNames.length;
        releaseActiveMujocoHandles();
        return { ok: true, label, joints };
      } catch (e) {
        return { ok: false, label, error: e?.message ?? String(e) };
      }
    };

    const results = [];
    results.push(await run('dump-failed', urdf, 'manual'));

    let base = urdf;
    try {
      const res = await fetch('/biped-assets/urdf/biped_s70_upper_body.urdf');
      if (res.ok) base = await res.text();
    } catch {
      // keep dump
    }
    const prepared = prepareUrdfForMujocoLoad(base);
    results.push(await run('biped-initial', prepared, 'initial'));

    const { urdfText: withPayload } = appendSpherePayloadWithRecord(prepared, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    results.push(await run('biped-payload', prepareUrdfForMujocoLoad(withPayload), 'payload-reload'));

    return results;
  }, dump);

  console.log('=== WASM loader tests ===');
  console.log(JSON.stringify(wasmResults, null, 2));

  if (pageErrors.length) {
    console.log('=== page errors ===');
    for (const e of pageErrors) console.log(e);
  }

  await page.close();

  const uiPage = await browser.newPage();
  await uiPage.goto(BASE_URL, { waitUntil: 'domcontentloaded', timeout: 120_000 });
  await uiPage.waitForFunction(
    () => window.__sessionStore?.getState().simStatus === 'ready',
    null,
    { timeout: 120_000 },
  );
  await uiPage.waitForTimeout(2000);

  const addBtn = uiPage.getByRole('button', { name: /添加球体/ });
  let uiError = false;
  let dumpAfterClick = false;
  if (await addBtn.count()) {
    const logsAfter = [];
    uiPage.on('console', (msg) => logsAfter.push(msg.text()));
    await addBtn.first().click();
    await uiPage.waitForTimeout(20000);
    dumpAfterClick = logsAfter.some((l) => l.includes('失败资源包已写入'));
    const bodyText = await uiPage.locator('body').innerText();
    uiError = bodyText.includes('MuJoCo 加载 URDF 失败');
    console.log('=== UI after add sphere ===');
    console.log('MuJoCo error visible:', uiError);
    console.log('Debug dump after click:', dumpAfterClick);
    if (uiError) {
      const errLine = bodyText.split('\n').find((l) => l.includes('MuJoCo 加载')) ?? '';
      console.log(errLine.slice(0, 400));
    }
  } else {
    console.log('=== UI: 未找到「添加球体」按钮（可能尚未加载完成）===');
  }

  if (consoleLogs.length > 0) {
    console.log('=== page console (tail) ===');
    for (const line of consoleLogs.slice(-25)) console.log(line);
  }

  await uiPage.close();
  await browser.close();

  const wasmOk = wasmResults.every((r) => r.ok);
  process.exit(wasmOk && !uiError && !dumpAfterClick ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
