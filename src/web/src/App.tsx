import Box from '@suid/material/Box'
import AppBar from '@suid/material/AppBar'
import CssBaseline from '@suid/material/CssBaseline'
import Typography from '@suid/material/Typography'
import Toolbar from '@suid/material/Toolbar'
import useTheme from '@suid/material/styles/useTheme'
import Managment from './Managment'
import { createEffect, createResource, onCleanup, Show } from 'solid-js'
import { status as apiStatus } from './api'
import Launch from './Launch'


export default function App() {
  const theme = useTheme()
  const [status, { refetch: refreshStatus }] = createResource(apiStatus)

  const intervalId = setInterval(() => refreshStatus(), 1000)
  onCleanup(() => clearInterval(intervalId))
  createEffect(() => console.log(status()))

  return (
    <Box sx={{ display: 'flex' }}>
      <CssBaseline />
      <AppBar
        position="fixed"
        sx={{
          get zIndex() {
            return theme.zIndex.drawer + 1
          },
        }}
      >
        <Toolbar>
          <Typography variant="h6" noWrap component="div">
            Cloversim WebUI
          </Typography>
        </Toolbar>
      </AppBar>
      <Show when={status()?.launch_status} fallback={<Managment />}>
        <Launch />
      </Show>
    </Box>
  )
}
