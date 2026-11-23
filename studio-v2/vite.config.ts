import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import {createHash} from 'node:crypto';
import {readdirSync, readFileSync} from 'node:fs';
import {resolve, relative} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url));
function modelFiles(folder:string):string[]{
  return readdirSync(folder,{withFileTypes:true}).flatMap(entry=>{
    const path=resolve(folder,entry.name);
    return entry.isDirectory()?modelFiles(path):/\.(py|json)$/.test(entry.name)?[path]:[];
  });
}
const digest=createHash('sha256');
for(const file of [...modelFiles(resolve(root,'mission_planner')),...modelFiles(resolve(root,'config/planner'))].sort()){
  digest.update(relative(root,file).replaceAll('\\','/')).update('\0').update(readFileSync(file)).update('\0');
}
export default defineConfig({
  define:{'import.meta.env.VITE_SOLVER_DIGEST':JSON.stringify(digest.digest('hex'))},
  plugins: [react()],
  base: '/',
  server: { host: '127.0.0.1', port: 5174, strictPort: true,
    proxy: { '/api': { target: 'http://127.0.0.1:8765', changeOrigin: true } } },
  build: { outDir: 'dist', emptyOutDir: true }
});
