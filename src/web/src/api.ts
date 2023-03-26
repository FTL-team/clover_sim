import { NodeError } from '../../lib/bindings/NodeError'
import { TaskInfo } from '../../lib/bindings/TaskInfo'
import { NodeStatus } from '../../lib/bindings/NodeStatus'
import { LaunchEvent } from '../../lib/bindings/LaunchEvent'

type NodeResult<T> = { Ok: T } | { Err: NodeError }

export const rootUrl = new URL('../cloversim_api/', new URL(import.meta.url))
console.log(rootUrl)

function unwrapNodeResult<T>(result: NodeResult<T>): T {
  if ('Err' in result) {
    throw new Error(JSON.stringify(result.Err))
  }
  return result.Ok
}

export async function status(): Promise<NodeStatus> {
  let res = await fetch(new URL(`./status`, rootUrl))
  let jres = (await res.json()) as NodeResult<NodeStatus>
  return unwrapNodeResult(jres)
}

export async function listWorkspaces(): Promise<string[]> {
  let res = await fetch(new URL(`./list_workspaces`, rootUrl))
  let jres = (await res.json()) as NodeResult<string[]>
  return unwrapNodeResult(jres)
}

export async function listTasks(): Promise<TaskInfo[]> {
  let res = await fetch(new URL(`./list_tasks`, rootUrl))
  let jres = (await res.json()) as NodeResult<TaskInfo[]>
  return unwrapNodeResult(jres)
}

export async function createWorkspace(name: string): Promise<[]> {
  let res = await fetch(new URL(`./create_workspace`, rootUrl), {
    method: 'POST',
    body: JSON.stringify(name),
    headers: {
      'Content-Type': 'application/json',
    },
  })
  let jres = (await res.json()) as NodeResult<[]>
  return unwrapNodeResult(jres)
}

export async function removeWorkspace(name: string): Promise<[]> {
  let res = await fetch(new URL(`./remove_workspace`, rootUrl), {
    method: 'POST',
    body: JSON.stringify(name),
    headers: {
      'Content-Type': 'application/json',
    },
  })
  let jres = (await res.json()) as NodeResult<[]>
  return unwrapNodeResult(jres)
}

export async function cloneWorkspace(
  name: string,
  newName: string
): Promise<[]> {
  let res = await fetch(new URL(`./duplicate_workspace`, rootUrl), {
    method: 'POST',
    body: JSON.stringify([name, newName]),
    headers: {
      'Content-Type': 'application/json',
    },
  })
  let jres = (await res.json()) as NodeResult<[]>
  return unwrapNodeResult(jres)
}

export async function launch(name: string, task: string): Promise<[]> {
  let res = await fetch(new URL(`./launch`, rootUrl), {
    method: 'POST',
    body: JSON.stringify([name, task]),
    headers: {
      'Content-Type': 'application/json',
    },
  })
  let jres = (await res.json()) as NodeResult<[]>
  return unwrapNodeResult(jres)
}

export async function fetchTaskFile(file = 'README.md') {
  let res = await fetch(new URL(`./task/${file}`, rootUrl))
  return await res.text()
}

export class LaunchConnection {
  onMessage: (msg: LaunchEvent) => void
  socket: WebSocket | null
  refreshInterval: number | null

  constructor() {
    this.onMessage = (msg) => {}
    this.socket = null
  }

  connect() {
    const url = new URL('./launch_channel', rootUrl)
    url.protocol = url.protocol.replace('http', 'ws')
    const socket = new WebSocket(url)

    socket.addEventListener('error', (event) => {
      setTimeout(() => this.connect(), 500)
    })

    socket.addEventListener('open', (event) => {
      this.send('RefreshScores')
      this.send('RefreshRand')
    })

    socket.addEventListener('message', (event) => {
      this.onMessage(JSON.parse(event.data))
    })

    this.socket = socket
  }

  disconnect() {
    if (this.socket) {
      this.socket.close()
      this.socket = null
      clearInterval(this.refreshInterval)
    }
  }

  send(ev: LaunchEvent) {
    if (this.socket) {
      this.socket.send(JSON.stringify(ev))
    }
  }
}
