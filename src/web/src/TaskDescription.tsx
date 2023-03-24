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

const fetchReadme = () => fetchTaskFile()

marked.setOptions({
  highlight: function (code, lang) {
    const language = hljs.getLanguage(lang) ? lang : 'plaintext'
    return hljs.highlight(code, { language }).value
  },
  langPrefix: 'hljs language-',
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
