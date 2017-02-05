Plots the lighthouse positions, and the hmd bearing angles from stdin using OpenGL

----
- Step 1:
----

You must run lighthousefind first

lighthousefind L processed_data.txt > L.txt 
lighthousefind R processed_data.txt > R.txt

you must move L.txt and R.txt into the same directory as this program

----
- Step 2
----

Then you need to run processrawcap and pipe the output into plot_lighthouse

../../tools/process_rawcap/process_to_points [raw data.csv] | ./plot_lighthouse
