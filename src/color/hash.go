package color

import (
	"hash/fnv"
)

var HASH_COLORS = []string{
	RED_ESC,
	GREEN_ESC,
	YELLOW_ESC,
	BLUE_ESC,
	PURPLE_ESC,
	CYAN_ESC,
}

func ColorHash(s string) string {
	h := fnv.New32a()
	h.Write([]byte(s))
	
	ci := h.Sum32() % uint32(len(HASH_COLORS))
	return HASH_COLORS[ci]
}