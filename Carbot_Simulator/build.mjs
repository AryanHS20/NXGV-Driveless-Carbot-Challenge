import {build} from 'esbuild';
await build({entryPoints:['src/app.js'],bundle:true,format:'iife',outfile:'app.bundle.js',minify:true,sourcemap:false,legalComments:'eof'});
console.log('Bundled all runtime code; no internet needed to run.');
