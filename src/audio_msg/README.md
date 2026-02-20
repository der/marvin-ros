## Audio stream formats

Loosely based on audio_common but with simplifications. Turns out reasons for ditching audio_common are invalid so maybe should go back to that.

`AudioInfo` - stream info which defaults to 16kHz, mono, raw, 512 chunk size
`AudioData` - both `int16_data` and `float32_data` allowed
`Audio` - single packet of info plus data
`AudioStamped` - packet with standard header that supplies timestamp and frame id
