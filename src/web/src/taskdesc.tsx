/* @refresh reload */
import { render } from 'solid-js/web'
import { TaskDescription } from './TaskDescription'

import '@fontsource/roboto/300.css'
import '@fontsource/roboto/400.css'
import '@fontsource/roboto/500.css'
import '@fontsource/roboto/700.css'
import '@fontsource/roboto-mono/400.css'

render(() => <TaskDescription />, document.getElementById('root')!)
