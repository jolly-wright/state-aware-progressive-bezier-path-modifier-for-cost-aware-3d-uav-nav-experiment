# A state-aware progressive B\'ezier path modifier for cost-aware 3D UAV navigation
A downstream progressive bezier modifier that modifies waypoint based paths from upstream planners with proper data output format, modifies by going through collision and objective wighted cost fuctions. This is still an experimental stage, experimented on Teensy 4.0, not recommended for direct usage unless one knows what they are doing.

1. you can input scenarios and their objects via py_scripts/input.py, run that file as it's going to create required json files in scenarios folder.
2. this is a platform io project, so u can directly upload this to a Teensy or change environment in platformio.ini
3. cost weights are set in include/config.hpp
4. run.py reads json files from scenarios folder and asks confirmation msg for running them (if the terminal resets, the py script may not find the teensy, unplug and replug it without resetting the terminal), run.py communicates with the teensy via serial usb
5. all the scenarios are run through teensy, if there are impossible scenarios, the code might crash and not output will be provided by the teensy, thus check 'input_visualization' folder after running input.py as it'll generate html files of 3d plots for each scenarios for you to view and debug scenario inputs [make sure to have required python dependencies such as matplotlib and plotly, both are used here]
6. successful run on the teensy would create json file results by the run.py in teensy_results/ directory. u can safely unplug the teensy
7. running output.py after those json files are created, will also create html files for each scenario in output_visualization/ directory to view outputs in 3d plots
8. This is only in experimental stage so again, not recommended for global usage unless one knows what he's doing
9. don't forget to crashout if u didn't understand anything
10. have fun!
