# Course slides — how the ESP32 drone works

A 150-slide course for new learners: how a quadcopter flies, every part on the
board, the firmware design, and the code line by line — with examples, diagrams
and "ask why" questions in every section.

| | English | Tiếng Việt |
|---|---|---|
| PDF (read on GitHub or print) | [ESP32_Drone_Course_EN.pdf](ESP32_Drone_Course_EN.pdf) | [ESP32_Drone_Course_VI.pdf](ESP32_Drone_Course_VI.pdf) |
| Web page (download, open in a browser) | [ESP32_Drone_Course_EN.html](ESP32_Drone_Course_EN.html) | [ESP32_Drone_Course_VI.html](ESP32_Drone_Course_VI.html) |
| Slide sources | [source/en](source/en) | [source/vi](source/vi) |

Speaker notes are in each slide's `<aside>` in the source files (they are not
shown in the PDF).

## Sections

1. Welcome · 2. How a quadcopter flies · 3. The hardware parts ·
4. Firmware architecture · 5. Reading the motion sensor · 6. PID control ·
7. Mixer and motors · 8. Radio inputs: SBUS and CRTP · 9. Flight modes and
safety · 10. Navigation · 11. Testing and workflow · 12. Wrap-up

Every code slide quotes [`Esp32_drone/Esp32_drone.ino`](../../Esp32_drone/Esp32_drone.ino);
the footer names the function. If a slide and the code ever disagree, the code
wins. The PID, filter and instability curves are illustrations of the idea,
not recordings; numbers taken from the firmware (gains, limits, the mixer
example, the leak curve) are exact.

## Source format

`source/<lang>/deck.json` lists the slide order and sections;
`source/<lang>/slides/sNNN.html` holds one 1920×1080 slide each, inline styles
only, with diagrams as inline SVG.
