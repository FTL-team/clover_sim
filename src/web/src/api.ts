import { NodeError } from '../../lib/bindings/NodeError'
import { TaskInfo } from '../../lib/bindings/TaskInfo'
import { NodeStatus } from '../../lib/bindings/NodeStatus'
import { LaunchEvent } from '../../lib/bindings/LaunchEvent'

type NodeResult<T> = { Ok: T } | { Err: NodeError }

function unwrapNodeResult<T>(result: NodeResult<T>): T {
  if ('Err' in result) {
    throw new Error(JSON.stringify(result.Err))
  }
  return result.Ok
}

export async function status(): Promise<NodeStatus> {
  let res = await fetch(`./cloversim_api/status`)
  let jres = (await res.json()) as NodeResult<NodeStatus>
  return unwrapNodeResult(jres)
}

export async function listWorkspaces(): Promise<string[]> {
  let res = await fetch(`./cloversim_api/list_workspaces`)
  let jres = (await res.json()) as NodeResult<string[]>
  return unwrapNodeResult(jres)
}

export async function listTasks(): Promise<TaskInfo[]> {
  let res = await fetch(`./cloversim_api/list_tasks`)
  let jres = (await res.json()) as NodeResult<TaskInfo[]>
  return unwrapNodeResult(jres)
}

export async function createWorkspace(name: string): Promise<[]> {
  let res = await fetch(`./cloversim_api/create_workspace`, {
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
  let res = await fetch(`./cloversim_api/remove_workspace`, {
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
  let res = await fetch(`./cloversim_api/duplicate_workspace`, {
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
  let res = await fetch(`./cloversim_api/launch`, {
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
  let res = await fetch(`./cloversim_api/task/${file}`)
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
    const url = new URL('./cloversim_api/launch_channel', window.location.href)
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
    this.socket.close()
    this.socket = null
    clearInterval(this.refreshInterval)
  }

  send(ev: LaunchEvent) {
    if (this.socket) {
      this.socket.send(JSON.stringify(ev))
    }
  }
}
