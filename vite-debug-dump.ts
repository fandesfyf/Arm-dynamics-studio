import type { Plugin } from 'vite';
import { mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname } from 'node:path';

const DUMP_DIR = join(process.cwd(), 'debug-dumps');

interface MeshEntry {
  path: string;
  base64: string;
}

interface BundleBody {
  folderName?: string;
  urdfText?: string;
  urdfFileName?: string;
  detail?: string;
  loadPhase?: string;
  rawUrdfLength?: number;
  hasPayload?: boolean;
  meshCount?: number;
  meshes?: MeshEntry[];
}

function writeRobotBundle(body: BundleBody): string {
  const folderName =
    (body.folderName ?? `mujoco-failed-${Date.now()}`).replace(/[^\w.-]/g, '_');
  const root = join(DUMP_DIR, folderName);
  const urdfRel = (body.urdfFileName ?? 'urdf/robot.urdf').replace(/^\/+/, '');
  const urdfPath = join(root, urdfRel);
  mkdirSync(dirname(urdfPath), { recursive: true });
  writeFileSync(urdfPath, body.urdfText ?? '', 'utf-8');

  for (const mesh of body.meshes ?? []) {
    const meshPath = join(root, mesh.path.replace(/^\/+/, ''));
    mkdirSync(dirname(meshPath), { recursive: true });
    writeFileSync(meshPath, Buffer.from(mesh.base64, 'base64'));
  }

  writeFileSync(join(root, 'error.txt'), body.detail ?? '', 'utf-8');
  const readme = [
    'MuJoCo 加载失败调试包',
    `时间: ${new Date().toISOString()}`,
    `阶段: ${body.loadPhase ?? 'unknown'}`,
    `含负载 link: ${body.hasPayload ? '是' : '否'}`,
    `原始 URDF 长度: ${body.rawUrdfLength ?? 0}`,
    `导出 mesh 数: ${body.meshCount ?? body.meshes?.length ?? 0}`,
    '',
    '手动验证（MuJoCo simulate / Python）:',
    `  URDF: ${urdfRel}`,
    '  mesh 路径与 URDF 中 filename 一致（../meshes/ 相对 urdf 目录）',
    '',
    '流程对比:',
    '  initial: App 启动 -> loadDefaultBiped -> fetch URDF+mesh -> loadRobot',
    '  payload-reload: 添加球体 -> appendSpherePayload -> reloadUrdf -> loadRobot(内存 mesh)',
    '',
    `错误: ${body.detail ?? ''}`,
  ].join('\n');
  writeFileSync(join(root, 'README.txt'), readme, 'utf-8');

  return root;
}

/** 开发环境：MuJoCo 失败时写入 web/debug-dumps/<bundle>/ */
export function debugDumpPlugin(): Plugin {
  return {
    name: 'debug-dump',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (req.method !== 'POST') {
          next();
          return;
        }

        if (req.url === '/__debug/dump-robot-bundle') {
          const chunks: Buffer[] = [];
          req.on('data', (chunk: Buffer) => chunks.push(chunk));
          req.on('end', () => {
            try {
              const body = JSON.parse(Buffer.concat(chunks).toString('utf-8')) as BundleBody;
              mkdirSync(DUMP_DIR, { recursive: true });
              const path = writeRobotBundle(body);
              res.setHeader('Content-Type', 'application/json');
              res.end(JSON.stringify({ ok: true, path }));
            } catch (e) {
              res.statusCode = 500;
              res.setHeader('Content-Type', 'application/json');
              res.end(
                JSON.stringify({
                  ok: false,
                  error: e instanceof Error ? e.message : String(e),
                }),
              );
            }
          });
          return;
        }

        if (req.url === '/__debug/dump-urdf') {
          const chunks: Buffer[] = [];
          req.on('data', (chunk: Buffer) => chunks.push(chunk));
          req.on('end', () => {
            try {
              const body = JSON.parse(Buffer.concat(chunks).toString('utf-8')) as {
                filename?: string;
                urdfText?: string;
                detail?: string;
              };
              const path = writeRobotBundle({
                folderName: (body.filename ?? `mujoco-failed-${Date.now()}.urdf`).replace(
                  /\.urdf$/i,
                  '',
                ),
                urdfText: body.urdfText,
                urdfFileName: 'urdf/robot.urdf',
                detail: body.detail,
                loadPhase: 'legacy-single-urdf',
                meshes: [],
              });
              res.setHeader('Content-Type', 'application/json');
              res.end(JSON.stringify({ ok: true, path }));
            } catch (e) {
              res.statusCode = 500;
              res.setHeader('Content-Type', 'application/json');
              res.end(
                JSON.stringify({
                  ok: false,
                  error: e instanceof Error ? e.message : String(e),
                }),
              );
            }
          });
          return;
        }

        next();
      });
    },
  };
}
