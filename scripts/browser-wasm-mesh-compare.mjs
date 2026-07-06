/**
 * 对比有/无 mesh VFS 的 WASM 加载。
 */
import { chromium } from 'playwright';

const BASE_URL = process.env.APP_URL ?? 'http://localhost:5173/';

async function main() {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage();
  await page.goto(BASE_URL, { waitUntil: 'domcontentloaded' });
  await page.waitForTimeout(2000);

  const results = await page.evaluate(async () => {
    const { loadMujocoRobot } = await import('/src/mujoco/loader.ts');
    const { prepareUrdfForMujocoLoad } = await import('/src/utils/urdf-sanitize.ts');
    const { appendSpherePayloadWithRecord } = await import('/src/core/payload-editor.ts');

    const res = await fetch('/biped-assets/urdf/biped_s70_upper_body.urdf');
    const urdf = await res.text();
    const prepared = prepareUrdfForMujocoLoad(urdf);
    const { urdfText: withPayload } = appendSpherePayloadWithRecord(prepared, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const payloadUrdf = prepareUrdfForMujocoLoad(withPayload);

    const meshMap = new Map();
    const refs = [...payloadUrdf.matchAll(/filename="([^"]+\.STL)"/gi)].map((m) => m[1]);
    for (const ref of refs) {
      const base = ref.split('/').pop();
      const url = `/biped-assets/meshes/${base}`;
      const r = await fetch(url);
      if (!r.ok) continue;
      const bytes = new Uint8Array(await r.arrayBuffer());
      meshMap.set(ref, bytes);
      meshMap.set(ref.replace(/^\.\.\//, ''), bytes);
      meshMap.set(`meshes/${base}`, bytes);
      meshMap.set(base, bytes);
    }

    const run = async (label, text, meshes) => {
      try {
        const bundle = await loadMujocoRobot({
          urdfText: text,
          urdfFileName: 'urdf/biped_s70_upper_body.urdf',
          meshes,
          loadPhase: 'payload-reload',
        });
        const n = bundle.jointNames.length;
        bundle.model.delete();
        bundle.data.delete();
        return { ok: true, label, n, meshCount: meshes.size };
      } catch (e) {
        return { ok: false, label, error: e?.message?.split('\n')[0] ?? String(e), meshCount: meshes.size };
      }
    };

    return [
      await run('payload-no-mesh', payloadUrdf, new Map()),
      await run('payload-with-mesh', payloadUrdf, meshMap),
      await run('initial-with-mesh', prepared, meshMap),
    ];
  });

  console.log(JSON.stringify(results, null, 2));
  await browser.close();
  process.exit(results.every((r) => r.ok) ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
