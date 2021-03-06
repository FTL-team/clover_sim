package main

import (
	"fmt"
	"os"
	"sync"

	"github.com/FTL-team/clover_sim/src/color"
)

type LogLevel int
var logLevel LogLevel
const (
	ERROR_LOGLEVEL LogLevel = -1
	INFO_LOGLEVEL LogLevel = 0
	VERBOSE_LOGLEVEL LogLevel = 1
)

var logMutex sync.Mutex

var PromptString string

type Logger struct {
	Machine string
	MachinePrefix string
}

func NewLogger(machine string) *Logger {
	return &Logger{
		Machine: machine,
		MachinePrefix: color.ColorHash(machine) + machine + color.RESET_ESC,
	}
}


const (
	ERROR_STRING   = " ERROR "
	INFO_STRING    = " INFO  "
	VERBOSE_STRING = "VERBOSE"
)

const (
	ERROR_COLOR   = color.RED_ESC
	INFO_COLOR    = color.CYAN_ESC
	VERBOSE_COLOR = color.GRAY_ESC
)

const (
	ERROR_PREFIX = ERROR_COLOR + ERROR_STRING + color.RESET_ESC
	INFO_PREFIX  = INFO_COLOR + INFO_STRING + color.RESET_ESC
	VERBOSE_PREFIX = VERBOSE_COLOR + VERBOSE_STRING + color.RESET_ESC
)

func (l *Logger) Error(format string, v ...interface{}) {
	l.Eprintf(ERROR_PREFIX, format, v...)
}

func (l *Logger) Info(format string, v ...interface{}) {
	if logLevel >= INFO_LOGLEVEL {
		l.Printf(INFO_PREFIX, format, v...)
	}
}


func (l *Logger) Verbose(format string, v ...interface{}) {
	if logLevel >= VERBOSE_LOGLEVEL {
		l.Printf(VERBOSE_PREFIX, format, v...)
	}
}

func (l *Logger) Printf(prefix string, format string, v ...interface{}) {
	s := fmt.Sprintf(format, v...)
	l.Print(prefix, s)
}

func (l *Logger) Print(prefix string, s string) {
	logMutex.Lock()
	promptClean()
	fmt.Printf("[ %s ][%s]: %s\n\r", prefix, l.MachinePrefix, s)
	promptShow()
	logMutex.Unlock()
}


func (l *Logger) Eprintf(prefix string, format string, v ...interface{}) {
	s := fmt.Sprintf(format, v...)
	l.Eprint(prefix, s)
}

func (l *Logger) Eprint(prefix string, s string) {
	logMutex.Lock()
	promptClean()
	fmt.Fprintf(os.Stderr, "[ %s ][%s]: %s\n\r", prefix, l.MachinePrefix, s)
	promptShow()
	logMutex.Unlock()
}

func promptClean() {
	fmt.Print("\x1b[1000D\x1b[2K")
}

func promptShow() {
	fmt.Print(PromptString)
}

func SetPrompt(prompt string) {
	if prompt == PromptString {
		return
	}

	logMutex.Lock()
	promptClean()
	PromptString = prompt
	promptShow()
	logMutex.Unlock()
}

func SetLogLevel(level LogLevel) {
	logLevel = level
}