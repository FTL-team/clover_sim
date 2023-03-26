import { resolve } from 'path'
import { defineConfig } from 'vite'
import suidPlugin from '@suid/vite-plugin'
import solidPlugin from 'vite-plugin-solid'
import cssInjectedByJsPlugin from 'vite-plugin-css-injected-by-js'

export default defineConfig({
  plugins: [
    suidPlugin(),
    solidPlugin(),
    cssInjectedByJsPlugin({
      jsAssetsFilterFunction: (c) => true
      ,
    }),
  ],
  build: {
    target: 'esnext',
    rollupOptions: {
      preserveEntrySignatures: 'strict',
      input: {
        main: resolve(__dirname, 'index.html'),
        simvsext: resolve(__dirname, 'simvsext.tsx'),
      },
      output: {
        entryFileNames(chunkInfo) {
          // console.log(chunkInfo)
          return `${chunkInfo.name}.js`
        },
      },
    },
  },

  server: {
    port: 3008,
    proxy: {
      '/cloversim_api': {
        target: 'http://localhost:7777',
        changeOrigin: true,
        ws: true,
      },
    },
  },
})
