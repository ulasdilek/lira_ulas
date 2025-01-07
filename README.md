# Project Overview

This project involves solving various robotic puzzles using different strategies. The puzzles are defined in `.g` files located in the `puzzles` directory. The project includes several scripts to solve these puzzles and analyze the results.

## Project Structure

- `puzzles/`: Contains the puzzle definition files.
- `results.csv`: Stores the results of the puzzle-solving strategies.
- `requirements.txt`: Lists the dependencies required for the project.
- `solver.py`: Main script to run the solver on all puzzles.
- `solution_v2-bottleneck.ipynb`: Jupyter notebook implementing the bottleneck strategy.
- `solution_v3-obstacle.ipynb`: Jupyter notebook implementing the obstacle strategy.
- `make_gif.py`: Script to create GIFs from PNG images.
- `utility.py`: Contains utility functions used across the project.
- `bottleneck_analysis.py`: Contains functions for bottleneck analysis.
- `config_to_graph.py`: Converts puzzle configurations to graph representations.

## Setup

1. **Install Dependencies**:
    Ensure you have Python installed. Then, install the required dependencies using:
    ```sh
    pip install -r requirements.txt
    ```

## Running the Solver

The `solver.py` script runs the solver on all puzzles using different strategies and records the results. **`solver.py` does not display the configurations or solutions!**

```sh
python solver.py
```

## Running the Jupyter Notebooks

### Solution V2 - Bottleneck Strategy

1. Open the Jupyter notebook:
    ```sh
    jupyter notebook solution_v2-bottleneck.ipynb
    ```
2. Set the puzzle at the top of the file
3. You can turn helper displays on/off by commenting display lines in the `forward_subproblem_search()` definition
4. After the solution path is found, we can animate the solution by running the final loop. You can choose to save the PNGs to create GIF of the solution later

### Solution V3 - FPR Strategy

1. Open the Jupyter notebook:
    ```sh
    jupyter notebook solution_v3-obstacle.ipynb
    ```
2. Set the puzzle at the top of the file
3. You can turn helper displays on/off by commenting display lines in the `forward_subproblem_search()` definition
4. After the solution path is found, we can animate the solution by running the final loop. You can choose to save the PNGs to create GIF of the solution later

## Creating GIFs

The `make_gif.py` script converts a sequence of PNG images into a GIF.
(remember to remove the contents of "temp/" after creating the GIF)

```sh
python make_gif.py <source_folder> <target_gif> [duration] [loop]
```

- `source_folder`: Path to the folder containing PNG files.
- `target_gif`: Path to the output GIF file.
- `duration` (optional): Time between frames in milliseconds (default is 100).
- `loop` (optional): Number of loops (0 for infinite).

Example:
```sh
python make_gif.py temp/ saves/output.gif 25 0
```

## Additional Information

- The `results.csv` file will be updated with the results of the solver runs.
- The `utility.py`, `bottleneck_analysis.py`, and `config_to_graph.py` scripts contain helper functions and classes used by the main scripts and notebooks.
