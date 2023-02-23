import {
  Alert,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  TextField,
} from '@suid/material'
import { AlertColor } from '@suid/material/Alert'
import { Component, createSignal } from 'solid-js'
import { DialogProps, IDialogContext } from './DialogContext'

type TextDialogProps = {
  title: string
  name: string
  resolve: (res: string | null) => void
}

const TextDialog: Component<DialogProps<TextDialogProps, string>> = (props) => {
  const [text, setText] = createSignal('')

  return (
    <Dialog
      open={true}
      onClose={() => {
        props.resolve(null)
        console.log('ON CLOSE')
      }}
      disableEscapeKeyDown={false}
      aria-labelledby="alert-dialog-title"
      aria-describedby="alert-dialog-description"
    >
      <DialogTitle id="alert-dialog-title">{props.title}</DialogTitle>
      <DialogContent>
        <TextField
          inputRef={(r) => {
            requestAnimationFrame(() => r.focus())
          }}
          onChange={(e, v) => setText(v)}
          value={text()}
          margin="dense"
          label={props.name}
          fullWidth
          variant="standard"
        />
      </DialogContent>
      <DialogActions>
        <Button onClick={() => props.resolve(null)}>Cancel</Button>
        <Button
          disabled={text() == ''}
          onClick={() => {
            props.resolve(text())
          }}
        >
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  )
}

export function askText(
  ctx: IDialogContext,
  title,
  name
): Promise<string | null> {
  console.log('Hello')
  return new Promise<string | null>((resolve) => {
    const props: TextDialogProps = {
      name,
      title,
      resolve,
    }

    ctx.addDialog({
      component: TextDialog,
      options: props,
    })
  })
}

type BoolDialogProps = {
  title: string
  name: string
  resolve: (res: boolean | null) => void
}

const BoolDialog: Component<DialogProps<BoolDialogProps, string>> = (props) => {
  return (
    <Dialog
      open={true}
      onClose={() => {
        props.resolve(false)
      }}
      disableEscapeKeyDown={false}
      aria-labelledby="alert-dialog-title"
      aria-describedby="alert-dialog-description"
    >
      <DialogTitle id="alert-dialog-title">{props.title}</DialogTitle>
      <DialogContent>
        <DialogContentText>{props.name}</DialogContentText>
      </DialogContent>
      <DialogActions>
        <Button onClick={() => props.resolve(false)}>No</Button>
        <Button
          onClick={() => {
            props.resolve(true)
          }}
        >
          Yes
        </Button>
      </DialogActions>
    </Dialog>
  )
}

export function askBool(
  ctx: IDialogContext,
  title,
  name
): Promise<boolean | null> {
  console.log('Hello')
  return new Promise<boolean | null>((resolve) => {
    const props: BoolDialogProps = {
      name,
      title,
      resolve,
    }

    ctx.addDialog({
      component: BoolDialog,
      options: props,
    })
  })
}

type AlertDialogProps = {
  err: string
  severity: AlertColor
  resolve: (res: null) => void
}

const SEVERITY_TITLES = {
  error: 'Error',
  success: 'Success',
  info: 'Info',
  warning: 'Warning',
}

const AlertDialog: Component<DialogProps<AlertDialogProps, string>> = (
  props
) => {
  return (
    <Dialog
      open={true}
      onClose={() => {
        props.resolve(null)
      }}
      disableEscapeKeyDown={false}
      aria-labelledby="alert-dialog-title"
      aria-describedby="alert-dialog-description"
    >
      <DialogTitle id="alert-dialog-title">
        {SEVERITY_TITLES[props.severity]}
      </DialogTitle>
      <DialogContent>
        <DialogContentText>
          <Alert
            severity={props.severity}
            style={{
              display: 'flex',
              'align-items': 'center',
            }}
          >
            <pre>{props.err}</pre>
          </Alert>
        </DialogContentText>
      </DialogContent>
      <DialogActions>
        <Button
          ref={(r) => requestAnimationFrame(() => r.focus())}
          onClick={() => {
            props.resolve(null)
          }}
        >
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  )
}

export function showAlert(
  ctx: IDialogContext,
  severity: AlertColor,
  err: string
): Promise<null> {
  console.log('Hello')
  return new Promise<null>((resolve) => {
    const props: AlertDialogProps = {
      err,
      severity,
      resolve,
    }

    ctx.addDialog({
      component: AlertDialog,
      options: props,
    })
  })
}
