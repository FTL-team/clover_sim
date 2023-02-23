import { defineConfig } from 'vite'
import suidPlugin from '@suid/vite-plugin'
import solidPlugin from 'vite-plugin-solid'

export default defineConfig({
  plugins: [suidPlugin(), solidPlugin()],
  build: {
    target: 'esnext',
  },

  server: {
    port: 3008,
    proxy: {
      '/cloversim_api': {
        target: 'http://127.0.0.1:7777',
        changeOrigin: true,
        ws: true,
      },
    },
  },
})
