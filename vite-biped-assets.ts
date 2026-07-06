import type { Plugin } from 'vite';
import { createReadStream, existsSync, statSync } from 'node:fs';
import { join, normalize } from 'node:path';

const BIPED_ROOT = '/home/fandes/biped_s70';

/** 开发环境将 /home/fandes/biped_s70 映射到 /biped-assets */
export function bipedAssetsPlugin(): Plugin {
  return {
    name: 'biped-assets',
    apply: 'serve',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const rawUrl = req.url?.split('?')[0] ?? '';
        if (!rawUrl.startsWith('/biped-assets')) {
          next();
          return;
        }

        const rel = decodeURIComponent(rawUrl.slice('/biped-assets'.length)).replace(/^\/+/, '');
        const filePath = normalize(join(BIPED_ROOT, rel));
        if (!filePath.startsWith(normalize(BIPED_ROOT)) || !existsSync(filePath)) {
          res.statusCode = 404;
          res.end('Not found');
          return;
        }
        if (statSync(filePath).isDirectory()) {
          res.statusCode = 404;
          res.end('Not found');
          return;
        }

        const ext = filePath.split('.').pop()?.toLowerCase();
        const types: Record<string, string> = {
          stl: 'model/stl',
          dae: 'model/vnd.collada+xml',
          urdf: 'application/xml',
        };
        res.setHeader('Content-Type', types[ext ?? ''] ?? 'application/octet-stream');
        createReadStream(filePath).pipe(res);
      });
    },
  };
}
