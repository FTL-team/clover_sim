import {
  createSignal,
  createContext,
  useContext,
  FlowComponent,
  Context,
  Component,
  For,
} from 'solid-js'
import { Dynamic, Portal } from 'solid-js/web'

export type DialogProps<T, E> = T & {
  resolve: (res: E | null) => void
}

type DialogEl<T> = {
  component: Component<DialogProps<T, any>>
  options: T
}

export interface IDialogContext {
  addDialog: (dialog: DialogEl<any>) => void
}

const defaultContext: IDialogContext = {
  addDialog: (dialog) => {},
}

const DialogContext: Context<IDialogContext> = createContext(defaultContext)

export const DialogProvider: FlowComponent = (props) => {
  const [dialogStack, setDialogStack] = createSignal<DialogEl<any>[]>([])

  const addDialog = (dialog: DialogEl<any>) => {
    setDialogStack((d) => [...d, dialog])
  }

  document.addEventListener('keyup', (e) => {
    if (e.key == 'Escape') {
      if (dialogStack().length > 0) {
        dialogStack()[dialogStack().length - 1].options.resolve(null)
        setDialogStack((d) => d.slice(0, -2))
      }
    }
  })

  return (
    <DialogContext.Provider
      value={{
        addDialog,
      }}
    >
      {props.children}
      <Portal>
        <For each={dialogStack()}>
          {(e) => (
            <Dynamic
              component={e.component}
              {...e.options}
              resolve={(res) => {
                e.options.resolve(res)
                setDialogStack((d) => d.slice(0, -2))
              }}
            />
          )}
        </For>
      </Portal>
    </DialogContext.Provider>
  )
}

export function useDialog() {
  return useContext(DialogContext)
}
