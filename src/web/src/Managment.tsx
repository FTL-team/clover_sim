import { createResource, createSignal, For } from 'solid-js'

import Box from '@suid/material/Box'
import Toolbar from '@suid/material/Toolbar'
import Typography from '@suid/material/Typography'
import Paper from '@suid/material/Paper'
import Button from '@suid/material/Button'
import Fab from '@suid/material/Fab'
import IconButton from '@suid/material/IconButton'
import Radio from '@suid/material/Radio'
import Table from '@suid/material/Table'
import TableBody from '@suid/material/TableBody'
import TableCell from '@suid/material/TableCell'
import TableHead from '@suid/material/TableHead'
import TableRow from '@suid/material/TableRow'
import AddIcon from '@suid/icons-material/Add'
import DeleteIcon from '@suid/icons-material/Delete'
import ContentCopyIcon from '@suid/icons-material/ContentCopy'
import RocketLaunchIcon from '@suid/icons-material/RocketLaunch'

import {
  cloneWorkspace as duplicateWorkspace,
  createWorkspace,
  launch,
  listTasks,
  listWorkspaces,
  removeWorkspace,
} from './api'
import { useDialog } from './DialogContext'
import { askBool, askText, showAlert } from './CommonDialogs'

export default function Managment() {
  const [workspaces, { refetch: refreshWorkspaces }] =
    createResource(listWorkspaces)
  const [tasks] = createResource(listTasks)
  const dialogCtx = useDialog()

  const [selectedWs, setSelectedWs] = createSignal<string | null>(null)
  const [selectedTask, setSelectedTask] = createSignal<string | null>(null)

  return (
    <Box
      component="main"
      sx={{
        flexGrow: 1,
        p: 3,
      }}
    >
      <Toolbar />
      <Box
        component="main"
        sx={{
          display: 'flex',
          flexGrow: 1,
          flexDirection: 'row',
          gap: 2,
        }}
      >
        <Paper
          elevation={3}
          sx={{
            padding: 2,
            flex: 1,
          }}
        >
          <Typography variant="h4" component="h1" gutterBottom>
            Workspaces
          </Typography>

          <Button
            startIcon={<AddIcon />}
            variant="text"
            onClick={async () => {
              const name = await askText(
                dialogCtx,
                'Create Workspace',
                'Workspace Name'
              )
              if (name == null) return
              try {
                await createWorkspace(name)
                showAlert(
                  dialogCtx,
                  'success',
                  `Workspace ${name} created successfully`
                )
                refreshWorkspaces()
              } catch (e) {
                showAlert(dialogCtx, 'error', e.message)
              }
            }}
          >
            New workspace
          </Button>

          <Table>
            <TableHead>
              <TableRow>
                <TableCell sx={{ p: 0, width: 0 }}></TableCell>
                <TableCell>Name</TableCell>
                <TableCell></TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              <For each={workspaces()}>
                {(ws) => (
                  <TableRow>
                    <TableCell sx={{ p: 0, width: 0 }}>
                      <Radio
                        checked={selectedWs() == ws}
                        onChange={() => setSelectedWs(ws)}
                      />
                    </TableCell>
                    <TableCell>{ws}</TableCell>
                    <TableCell align="right">
                      <IconButton
                        onClick={async () => {
                          const newName = await askText(
                            dialogCtx,
                            'Duplicate Workspace',
                            'New Workspace Name'
                          )
                          if (newName == null) return
                          try {
                            await duplicateWorkspace(ws, newName)
                            showAlert(
                              dialogCtx,
                              'success',
                              `Workspace ${ws} duplicated into ${newName} successfully`
                            )
                            refreshWorkspaces()
                          } catch (e) {
                            showAlert(dialogCtx, 'error', e.message)
                          }
                        }}
                      >
                        <ContentCopyIcon />
                      </IconButton>

                      <IconButton
                        onClick={async () => {
                          const value = await askBool(
                            dialogCtx,
                            'Remove workspace',
                            `Are you sure you want to remove workspace ${ws}`
                          )
                          if (value !== true) return
                          try {
                            await removeWorkspace(ws)
                            showAlert(
                              dialogCtx,
                              'success',
                              `Workspace ${ws} removed successfully`
                            )
                            refreshWorkspaces()
                          } catch (e) {
                            showAlert(dialogCtx, 'error', e.message)
                          }
                        }}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </TableCell>
                  </TableRow>
                )}
              </For>
            </TableBody>
          </Table>
        </Paper>

        <Paper
          elevation={3}
          sx={{
            padding: 2,
            flex: 1,
          }}
        >
          <Typography variant="h4" component="h1" gutterBottom>
            Tasks
          </Typography>

          <Table>
            <TableHead>
              <TableRow>
                <TableCell sx={{ p: 0, width: 0 }}></TableCell>
                <TableCell>Path</TableCell>
                <TableCell>Name</TableCell>
                <TableCell>Description</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              <For each={tasks()}>
                {(task) => (
                  <TableRow>
                    <TableCell sx={{ p: 0, width: 0 }}>
                      <Radio
                        checked={selectedTask() == task.relpath}
                        onChange={() => setSelectedTask(task.relpath)}
                      />
                    </TableCell>
                    <TableCell>{task.relpath}</TableCell>
                    <TableCell>{task.name}</TableCell>
                    <TableCell>{task.description}</TableCell>
                  </TableRow>
                )}
              </For>
            </TableBody>
          </Table>
        </Paper>
      </Box>

      <Fab
        color="primary"
        aria-label="add"
        disabled={selectedTask() == null || selectedWs() == null}
        sx={{
          position: 'fixed',
          bottom: 16,
          right: 16,
        }}
        onClick={async () => {
          try {
            await launch(selectedWs(), selectedTask())
          } catch (e) {
            showAlert(dialogCtx, 'error', e.message)
          }
        }}
      >
        <RocketLaunchIcon />
      </Fab>
    </Box>
  )
}
