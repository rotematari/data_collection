# Simple fire-and-forget command
uint8 mode
uint8 OPEN=1
uint8 CLOSE=2
uint8 STOP=3

# Optional knobs (unused for STOP). Leave zero if you don't care.
float32 position      # desired jaw position (m or rad, controller units)
float32 max_effort    # limit (controller units), 0 => ignore/use default
