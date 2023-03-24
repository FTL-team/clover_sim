import { Component, createMemo, For, Show } from 'solid-js'
import { TaskScore } from '../../lib/bindings/TaskScore'

interface ScoreTheme {
  textPrimary: string
  textSecondary: string
  textSuccess: string
  textError: string,
}

export const Score: Component<{ score: TaskScore; theme: ScoreTheme }> = (p) => {
  const flatScore = createMemo(() => {
    function flattenScore(score: TaskScore, depth = 0): [number, TaskScore][] {
      return [
        [depth, score],
        ...score.children.map((s) => flattenScore(s, depth + 1)).flat(),
      ]
    }
    if (p.score === null) return []
    return flattenScore(p.score)
  })

  return (
    <>
      <table>
        <tbody>
          <For each={flatScore()}>
            {(e) => {
              let color = p.theme.textPrimary
              if (e[1].score == 0) color = p.theme.textSecondary
              if (Math.abs(e[1].score - e[1].max_score) < 0.01)
                color = p.theme.textSuccess

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
                      color: p.theme.textPrimary,
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
                          color: p.theme.textError,
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
    </>
  )
}
