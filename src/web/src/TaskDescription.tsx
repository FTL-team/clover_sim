import DOMPurify from 'dompurify'
import { marked } from 'marked'
import { Component, createEffect, createResource, createSignal } from 'solid-js'
import { fetchTaskFile, rootUrl } from './api'
import styles from './TaskDescription.module.css'
import hljs from 'highlight.js/lib/core'
import 'highlight.js/styles/github-dark.css'

import python from 'highlight.js/lib/languages/python'
import javascript from 'highlight.js/lib/languages/javascript'
import c from 'highlight.js/lib/languages/c'
import cpp from 'highlight.js/lib/languages/cpp'
import xml from 'highlight.js/lib/languages/xml'
import yaml from 'highlight.js/lib/languages/yaml'

hljs.registerLanguage('python', python)
hljs.registerLanguage('javascript', javascript)
hljs.registerLanguage('cpp', cpp)
hljs.registerLanguage('c', c)
hljs.registerLanguage('xml', xml)
hljs.registerLanguage('yaml', yaml)

function attachCopyButton(el: HTMLElement) {
  let button = document.createElement('button')
  button.innerText = 'Copied!'
  button.className = styles['copy-button']
  el.appendChild(button)
  button.dataset['copied'] = 'false'

  // Add a custom proprety to the code block so that the copy button can reference and match its background-color value.
  el.style.setProperty(
    '--hljs-theme-background',
    window.getComputedStyle(el).backgroundColor
  )

  let curClearInterval = -1

  button.onclick = function () {
    if (!navigator.clipboard) return
    button.innerText = ''
    navigator.clipboard.writeText(el.innerText).then(() => {
      button.dataset['copied'] = 'true'
      button.innerText = 'Copied!'

      clearInterval(curClearInterval)
      curClearInterval = setTimeout(
        () => (button.dataset['copied'] = 'false'),
        1000
      )
    })
  }
}

function attachCopyButtonToAll() {
  let els = document.querySelectorAll<HTMLElement>('.hljs')
  for (let el of els) {
    if (el.querySelector('.' + styles['copy-button'])) continue
    attachCopyButton(el)
  }
}

const fetchReadme = () => fetchTaskFile()

marked.setOptions({
  highlight: function (code, lang) {
    const language = hljs.getLanguage(lang) ? lang : 'plaintext'
    console.log(hljs.highlight(code, { language }).value)
    return hljs.highlight(code, { language }).value
  },
  langPrefix: styles['codeblock'] + ' hljs language-',
})

const Markdown: Component<{ children: string }> = (p) => {
  let [ref, setRef] = createSignal<HTMLDivElement | null>(null)

  createEffect(() => {
    if (ref()) {
      let markdown = p.children
      let parsed = marked.parse(markdown, {
        baseUrl: new URL('./task/', rootUrl).toString(),
      })
      let safe = DOMPurify.sanitize(parsed)
      ref().innerHTML = safe
      attachCopyButtonToAll()
    }
  })

  return (
    <div
      class={styles.taskDescription}
      ref={(r) => {
        setRef(r)
      }}
    ></div>
  )
}

export const TaskDescription: Component = () => {
  const [taskReadme] = createResource(fetchReadme)

  return <Markdown>{taskReadme() ?? 'LOADING'}</Markdown>
}
