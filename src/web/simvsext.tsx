/* @refresh reload */
import { MountableElement, render } from 'solid-js/web'
import {
  provideVSCodeDesignSystem,
  vsCodeButton,
  vsCodeTextField,
} from '@vscode/webview-ui-toolkit'

import font from '@fontsource/roboto-mono/400.css?inline'
const fontEl = document.createElement('style')
fontEl.textContent = font.replaceAll(
  '/node_modules/',
  new URL('/node_modules/', new URL(import.meta.url)).toString()
)

document.head.appendChild(fontEl)

import ReplayIcon from '@suid/icons-material/Replay'
import PlayArrow from '@suid/icons-material/PlayArrow'
import Stop from '@suid/icons-material/Stop'
import { Score } from './src/Score'
import { createSignal, onCleanup, onMount } from 'solid-js'
import { LaunchConnection } from './src/api'
import { TaskScore } from '../lib/bindings/TaskScore'
import { TaskDescription } from './src/TaskDescription'

provideVSCodeDesignSystem().register(vsCodeButton(), vsCodeTextField())

declare module 'solid-js' {
  namespace JSX {
    interface IntrinsicElements {
      'vscode-badge': any
      'vscode-button': any
      'vscode-checkbox': any
      'vscode-data-grid': any
      'vscode-divider': any
      'vscode-dropdown': any
      'vscode-link': any
      'vscode-option': any
      'vscode-panels': any
      'vscode-progress-ring': any
      'vscode-radio': any
      'vscode-radio-group': any
      'vscode-tag': any
      'vscode-text-area': any
      'vscode-text-field': any
    }
  }
}

function Control() {
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

  let randDebounceTimeout = -1

  onMount(() => connection.connect())
  onCleanup(() => connection.disconnect())

  return (
    <>
      <div
        style={{
          display: 'flex',
          position: 'sticky',
          top: '0',
          left: '0',
          'align-items': 'center',
          gap: '10px',
          background: 'var(--vscode-editorGroupHeader-tabsBackground)',
          width: '100%',
          'box-sizing': 'border-box',
          'flex-wrap': 'wrap',
          padding: '10px',
        }}
      >
        <h1 style={{ margin: 0 }}>Simulator</h1>
        {/* <div
        style={{
          display: 'flex',
          gap: '10px',
          "flex-wrap": "wrap"
        }}
      > */}
        <vscode-button
          onClick={() => connection.send({ ControlSimulator: 'Start' })}
        >
          <div
            style={{
              display: 'flex',
              'flex-direction': 'row',
              // width: '48px',
              'justify-content': 'center',
              'align-items': 'center',
            }}
          >
            <PlayArrow />
            START
          </div>
        </vscode-button>
        <vscode-button
          onClick={() => connection.send({ ControlSimulator: 'Restart' })}
        >
          <div
            style={{
              display: 'flex',
              'flex-direction': 'row',
              // width: '48px',
              'justify-content': 'center',
              'align-items': 'center',
            }}
          >
            <ReplayIcon />
            RESTART
          </div>
        </vscode-button>
        <vscode-button
          onClick={() => connection.send({ ControlSimulator: 'Stop' })}
        >
          <div
            style={{
              display: 'flex',
              'flex-direction': 'row',
              // width: '48px',
              'justify-content': 'center',
              'align-items': 'center',
            }}
          >
            <Stop />
            STOP
          </div>
        </vscode-button>
        {/* </div> */}
        <div style={{ flex: 1 }} />
        <vscode-text-field
          value={rand()}
          onInput={(e) => {
            setRand(e.target.value)
            clearTimeout(randDebounceTimeout)
            randDebounceTimeout = setTimeout(() => {
              connection.send({ SetRand: rand() })
            }, 500)
          }}
        >
          <b>Randomization</b>
        </vscode-text-field>
      </div>
      <div
        style={{
          margin: '10px',
          'font-size': '1.2em',
        }}
      >
        <Score
          score={score()}
          theme={{
            textError: 'var(--vscode-errorForeground)',
            textPrimary: 'var(--vscode-editor-foreground)',
            textSecondary: 'var(--vscode-disabledForeground)',
            textSuccess: 'var(--vscode-terminal-ansiGreen)',
          }}
        />
      </div>
    </>
  )
}

export function attachControl(elemnt: Element) {
  return render(() => <Control />, elemnt)
}

export function attachDescription(elemnt: Element) {
  return render(() => <TaskDescription />, elemnt)
}
