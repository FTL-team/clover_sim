import {
  Box,
  Button,
  Divider,
  Fab,
  Paper,
  TextField,
  Toolbar,
  Typography,
  useTheme,
  ButtonGroup,
} from '@suid/material'
import {
  Component,
  createMemo,
  createSignal,
  For,
  onCleanup,
  onMount,
  Show,
} from 'solid-js'
import ReplayIcon from '@suid/icons-material/Replay'
import PlayArrow from '@suid/icons-material/PlayArrow'
import Stop from '@suid/icons-material/Stop'
import { LaunchConnection } from './api'
import { TaskScore } from '../../lib/bindings/TaskScore'
import PowerSettingsNewIcon from '@suid/icons-material/PowerSettingsNew'
import { askBool } from './CommonDialogs'
import { useDialog } from './DialogContext'
import { LaunchEvent } from '../../lib/bindings/LaunchEvent'
import Terminal from '@suid/icons-material/Terminal'
import { TaskDescription } from './TaskDescription'
import { Score } from './Score'

const GazeboIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48">
    <path
      fill="currentColor"
      d="M24 .002a1.36 1.36 0 0 0-.716.204L4.388 11.94a1.357 1.357 0 0 0-.641 1.154V34.91c0 .47.243.906.641 1.154l18.896 11.733.028.014c.01.005.02.014.03.02l.066.03.067.032c.035.014.071.027.107.038.022.007.042.015.064.02.04.013.082.02.123.027A1.238 1.238 0 0 0 24 48c.124 0 .248-.017.367-.05h.002c.046-.013.092-.031.136-.049l.041-.014c.038-.015.074-.039.11-.058l.06-.031v-.001h.001l.037-.023 18.857-11.71c.4-.248.643-.686.642-1.158l-.033-10.932a1.356 1.356 0 0 0-2.075-1.148l-8.044 5.01-6.2-3.84 15.71-9.784a1.36 1.36 0 0 0 .642-1.154 1.362 1.362 0 0 0-.643-1.153L24.715.204A1.355 1.355 0 0 0 23.999 0zM6.414 15.504 20.1 24.002 6.414 32.5zm18.915 10.094 6.201 3.84-8.279 5.155c-.4.248-.642.686-.64 1.157l.024 8.449-14.957-9.288L22.674 25.6l.61.38a1.353 1.353 0 0 0 1.434-.002zm16.185.821.022 7.739-16.185 10.05-.023-7.71z"
    />
  </svg>
)

const SimControl: Component<{
  control: (e: LaunchEvent) => void
  rand: string
  setRand: (n: string) => void
}> = (p) => {
  let randDebounceTimeout: number = -1

  return (
    <Paper
      sx={{
        display: 'flex',
        flexDirection: 'row',
        gap: 2,
      }}
    >
      <Box
        sx={{
          p: 1,
          flexGrow: 1,
        }}
      >
        <Typography variant="h5" component="h2">
          Simulator
        </Typography>
        <Box
          sx={{
            display: 'flex',
            flexDirection: 'row',
            justifyContent: 'space-around',
          }}
        >
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 80 }}
            onClick={() => p.control({ ControlSimulator: 'Start' })}
          >
            <PlayArrow sx={{ fontSize: 48, display: 'block' }} />
            Start
          </Button>
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 80 }}
            onClick={() => p.control({ ControlSimulator: 'Restart' })}
          >
            <ReplayIcon sx={{ fontSize: 48, display: 'block' }} />
            Restart
          </Button>
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 80 }}
            onClick={() => p.control({ ControlSimulator: 'Stop' })}
          >
            <Stop sx={{ fontSize: 48, display: 'block' }} />
            Stop
          </Button>
        </Box>
      </Box>
      <Divider
        sx={{ alignSelf: 'stretch', height: 'unset' }}
        orientation="vertical"
      />
      <Box
        sx={{
          flexGrow: 1,
          p: 1,
        }}
      >
        <Typography variant="h5" component="h2">
          Randomization
        </Typography>
        <Box sx={{ paddingTop: 1 }}>
          <TextField
            value={p.rand}
            onChange={(e) => {
              p.setRand(e.target.value)
              clearTimeout(randDebounceTimeout)
              randDebounceTimeout = setTimeout(() => {
                p.control({ SetRand: p.rand })
              }, 500)
            }}
            fullWidth
            label="Randomization seed"
            variant="standard"
          />
        </Box>
      </Box>
    </Paper>
  )
}

const ConnectWith: Component = () => {
  const theme = useTheme()
  const [mode, setMode] = createSignal('web')

  return (
    <Paper sx={{ p: 2 }}>
      <Box
        sx={{
          display: 'flex',
          flexDirection: 'row',
        }}
      >
        <Typography
          variant="h5"
          component="h2"
          gutterBottom
          sx={{ flex: 1, marginBottom: 0 }}
        >
          Connect with:
        </Typography>
        <ButtonGroup variant="outlined">
          <Button
            disableElevation
            variant={mode() == 'web' ? 'contained' : 'outlined'}
            onClick={() => setMode('web')}
          >
            Web
          </Button>
          <Button
            disableElevation
            variant={mode() == 'local' ? 'contained' : 'outlined'}
            onClick={() => setMode('local')}
          >
            Local SSH
          </Button>
          <Button
            disableElevation
            variant={mode() == 'remote' ? 'contained' : 'outlined'}
            onClick={() => setMode('remote')}
          >
            Remote SSH
          </Button>
        </ButtonGroup>
      </Box>
      <Show
        when={mode() == 'web'}
        fallback={() => (
          <Typography
            component="pre"
            sx={{
              marginTop: 2,
              fontFamily: '"Roboto Mono"',
              padding: 2,
              borderRadius: 2,
              background: theme.palette.grey[300],
            }}
          >
            {mode() == 'local'
              ? `ssh -X clover@192.168.77.10`
              : `ssh -X -o ProxyCommand='websocat -b - ${location.href.replace(
                  'http',
                  'ws'
                )}/ssh' clover@clover0`}
          </Typography>
        )}
      >
        <Box
          sx={{
            paddingTop: 2,
            display: 'flex',
            flexDirection: 'row',
            gap: 6,
            justifyContent: 'center',
          }}
        >
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 100 }}
            component="a"
            href="./ide/"
          >
            <span
              style={{
                'font-size': '40px',
                'line-height': '48px',
                display: 'block',
                'font-family': '"Roboto Mono"',
              }}
            >
              {'{}'}
            </span>
            IDE
          </Button>
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 100 }}
            component="a"
            href="./gzweb/"
          >
            <GazeboIcon />
            Gazebo
          </Button>
          <Button
            sx={{ display: 'flex', flexDirection: 'column', width: 100 }}
            component="a"
            href="./webterm/"
          >
            <Terminal sx={{ fontSize: 48, display: 'block' }} />
            Terminal
          </Button>
        </Box>
      </Show>
    </Paper>
  )
}

const ScorePaper: Component<{ score: TaskScore }> = (p) => {
  const theme = useTheme()

  return (
    <Paper sx={{ p: 2 }}>
      <Typography variant="h4" component="h1" gutterBottom>
        Task scoring
      </Typography>
      <Score
        score={p.score}
        theme={{
          textError: theme.palette.error.main,
          textSuccess: theme.palette.success.main,
          textPrimary: theme.palette.text.primary,
          textSecondary: theme.palette.text.secondary,
        }}
      />
    </Paper>
  )
}

const TaskDescriptionPaper: Component = () => {
  return (
    <Paper sx={{ p: 2 }}>
      <Typography variant="h4" component="h1" gutterBottom>
        Task description
      </Typography>
      <TaskDescription />
    </Paper>
  )
}

const Launch: Component = () => {
  const dialogCtx = useDialog()

  const [score, setScore] = createSignal<null | TaskScore>(null)
  const [rand, setRand] = createSignal<string>('0')

  const connection = new LaunchConnection()
  connection.onMessage = (message) => {
    if (message instanceof Object && 'Rand' in message) {
      setRand(message.Rand)
    }
    if (message instanceof Object && 'SetRand' in message) {
      setRand(message.SetRand)
    }
    if (message instanceof Object && 'TaskScores' in message) {
      setScore(message.TaskScores)
    }
  }

  onMount(() => connection.connect())
  onCleanup(() => connection.disconnect())

  return (
    <Box
      component="main"
      sx={{
        p: 3,
        flexGrow: 1,
      }}
    >
      <Toolbar />
      <Box
        sx={{ display: 'flex', flexDirection: 'row', gap: 2, flexWrap: 'wrap' }}
      >
        <Box
          sx={{
            flex: 1,
            display: 'flex',
            flexDirection: 'column',
            gap: 2,
            minWidth: '480px',
          }}
        >
          <SimControl
            control={(e) => connection.send(e)}
            rand={rand()}
            setRand={setRand}
          />
          <ConnectWith />
          <ScorePaper score={score()} />
        </Box>
        <Box sx={{ flex: 1, minWidth: '480px' }}>
          <TaskDescriptionPaper />
        </Box>
      </Box>
      <Fab
        color="primary"
        aria-label="add"
        sx={{
          position: 'fixed',
          bottom: 16,
          right: 16,
        }}
        onClick={async () => {
          let res = await askBool(
            dialogCtx,
            'Poweroff simulation?',
            'Are you sure you want to poweroff simulation?'
          )
          if (res !== true) return
          connection.send('Poweroff')
        }}
      >
        <PowerSettingsNewIcon />
      </Fab>
    </Box>
  )
}

export default Launch
