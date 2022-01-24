package color


const (
	RESET_ESC = "\x1b[0m"

	BLACK_ESC = "\x1b[30m"
	RED_ESC   = "\x1b[31m"
	GREEN_ESC = "\x1b[32m"
	YELLOW_ESC = "\x1b[33m"
	BLUE_ESC = "\x1b[34m"
	PURPLE_ESC = "\x1b[35m"
	CYAN_ESC = "\x1b[36m"
	WHITE_ESC = "\x1b[37m"

	GRAY_ESC = "\x1b[2;37m\x1b[38;5;248m"

	UNDERLINE_ESC = "\x1b[4m"
)

func Black(s string) string {
	return BLACK_ESC + s + RESET_ESC
}

func Red(s string) string {
	return RED_ESC + s + RESET_ESC
}

func Green(s string) string {
	return GREEN_ESC + s + RESET_ESC
}

func Yellow(s string) string {
	return YELLOW_ESC + s + RESET_ESC
}

func Blue(s string) string {
	return BLUE_ESC + s + RESET_ESC
}

func Purple(s string) string {
	return PURPLE_ESC + s + RESET_ESC
}

func Cyan(s string) string {
	return CYAN_ESC + s + RESET_ESC
}

func White(s string) string {
	return WHITE_ESC + s + RESET_ESC
}


func Underline(s string) string {
	return UNDERLINE_ESC + s + RESET_ESC
}
