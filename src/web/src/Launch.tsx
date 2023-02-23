import {
  Box,
  Button,
  Divider,
  Fab,
  Input,
  Paper,
  TextField,
  Toolbar,
  Typography,
  useTheme,
} from '@suid/material'
import {
  Component,
  createEffect,
  createMemo,
  createResource,
  createSignal,
  FlowComponent,
  For,
  onCleanup,
  onMount,
  ParentComponent,
  Show,
} from 'solid-js'
import ReplayIcon from '@suid/icons-material/Replay'
import PlayArrow from '@suid/icons-material/PlayArrow'
import Stop from '@suid/icons-material/Stop'
import { fetchTaskFile, LaunchConnection } from './api'
import { marked } from 'marked'
import DOMPurify from 'dompurify'
import { TaskScore } from '../../lib/bindings/TaskScore'
import PowerSettingsNewIcon from '@suid/icons-material/PowerSettingsNew'
import { askBool } from './CommonDialogs'
import { useDialog } from './DialogContext'

const fetchReadme = () => fetchTaskFile()

const Markdown: Component<{ children: string }> = (p) => {
  let [ref, setRef] = createSignal<HTMLDivElement | null>(null)

  createEffect(() => {
    if (ref()) {
      let markdown = p.children
      let parsed = marked.parse(markdown, { baseUrl: './cloversim_api/task/' })
      let safe = DOMPurify.sanitize(parsed)
      ref().innerHTML = safe
    }
  })

  return (
    <Box
      sx={{
        '*': {
          maxWidth: '100%',
        },
      }}
      ref={(r) => {
        setRef(r)
      }}
    ></Box>
  )
}

const Launch: Component = () => {
  const theme = useTheme()
  const dialogCtx = useDialog()

  const [taskReadme] = createResource(fetchReadme)
  const [rand, setRand] = createSignal('')
  const [score, setScore] = createSignal<null | TaskScore>(null)
  const flatScore = createMemo(() => {
    function flattenScore(score: TaskScore, depth = 0): [number, TaskScore][] {
      // console.log('SCORE', score)
      return [
        [depth, score],
        ...score.children.map((s) => flattenScore(s, depth + 1)).flat(),
      ]
    }
    if (score() === null) return []
    return flattenScore(score())
  })

  const connection = new LaunchConnection()
  connection.onMessage = (message) => {
    console.log('HANDLING MESSAGE:', message)
    if (message instanceof Object && 'Rand' in message) {
      setRand(message.Rand)
    }
    if (message instanceof Object && 'TaskScores' in message) {
      setScore(message.TaskScores)
    }
  }
  let randDebounceTimeout: number = -1

  onMount(() => connection.connect())
  onCleanup(() => connection.disconnect())

  createEffect(() => console.log(taskReadme()))
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
                  onClick={() => connection.send({ ControlSimulator: 'Start' })}
                >
                  <PlayArrow sx={{ fontSize: 48, display: 'block' }} />
                  Start
                </Button>
                <Button
                  sx={{ display: 'flex', flexDirection: 'column', width: 80 }}
                  onClick={() =>
                    connection.send({ ControlSimulator: 'Restart' })
                  }
                >
                  <ReplayIcon sx={{ fontSize: 48, display: 'block' }} />
                  Restart
                </Button>
                <Button
                  sx={{ display: 'flex', flexDirection: 'column', width: 80 }}
                  onClick={() => connection.send({ ControlSimulator: 'Stop' })}
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
                  value={rand()}
                  onChange={(e) => {
                    setRand(e.target.value)
                    clearTimeout(randDebounceTimeout)
                    randDebounceTimeout = setTimeout(() => {
                      connection.send({ SetRand: rand() })
                    }, 500)
                  }}
                  fullWidth
                  label="Randomization seed"
                  variant="standard"
                />
              </Box>
            </Box>
          </Paper>
          <Paper sx={{ p: 2 }}>
            <Typography variant="h4" component="h1" gutterBottom>
              Task scoring
            </Typography>
            <table>
              <tbody>
                <For each={flatScore()}>
                  {(e) => {
                    let color = theme.palette.text.primary
                    if (e[1].score == 0) color = theme.palette.text.secondary
                    if (Math.abs(e[1].score - e[1].max_score) < 0.01)
                      color = theme.palette.success.main

                    return (
                      <tr
                        style={{
                          'font-family': '"Roboto Mono"',
                          color,
                        }}
                      >
                        <td
                          style={{
                            'padding-left': `${e[0] * 10}px`,
                            width: '100%',
                            color: theme.palette.text.primary,
                          }}
                        >
                          {e[1].name}
                        </td>
                        <Show
                          when={!e[1].failed}
                          fallback={() => (
                            <td
                              colSpan={3}
                              style={{
                                'text-align': 'right',
                                color: theme.palette.error.main,
                              }}
                            >
                              failed
                            </td>
                          )}
                        >
                          <td style={{ 'text-align': 'right' }}>
                            {e[1].score.toFixed(2)}
                          </td>
                          <td>/</td>
                          <td>{e[1].max_score.toFixed(2)}</td>
                        </Show>
                      </tr>
                    )
                  }}
                </For>
              </tbody>
            </table>
          </Paper>
        </Box>
        <Box sx={{ flex: 1, minWidth: '480px' }}>
          <Paper sx={{ p: 2 }}>
            <Typography variant="h4" component="h1" gutterBottom>
              Task description
            </Typography>
            <Markdown>{taskReadme() ?? 'LOADING'}</Markdown>
          </Paper>
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
