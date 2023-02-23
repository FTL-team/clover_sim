/* @refresh reload */
import { render } from 'solid-js/web'
import App from './App'
import { DialogProvider } from './DialogContext'

import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import '@fontsource/roboto-mono/400.css';

render(
  () => (
    <DialogProvider>
      <App />
    </DialogProvider>
  ),
  document.getElementById('root')!
)
