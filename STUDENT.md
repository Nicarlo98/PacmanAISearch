# Student Information

**Course:** COMP90054 AI Planning for Autonomy

**Semester:** Semester 2, 2020

**Student:** 695788 - Mylan - Li - mylanl@student.unimelb.edu.au

* Your Student number - First Name - Last Name - Student email

Replace the lines above with your correct details.

Your student number should only be the **numbers**.


For part1, EHC.
Obvious difference is the helper function, efcImprove, which i did cause I implemented the code based off what was in the lecture slides.
A similarity is that a lot of the names are the same, since I copied the code from other sections of Search.py as a base, which i assume is written by the same person who did the solution.
Otherwise, the logic is all the same since EHC is still relatively a basic, fundamentals level algorithm (I think?), and since I just based it off what was in the lectures, I believe it to be the simplest way to do EHC.

For part2, IDA*.
Overall logic is the same. Again i based mine off the algorithm in the lecture slides, fundamental algorithm, simplest way to do IDA*.
One difference is that I use a list to keep track of nodes where the f is greater than the current limit in an iteration. This just came to me first as way to keep track of, and find the smallest of, values that were being generated. I don't think it has any functional impact; i don't think the computational power required is any different, but not sure.
Other difference is that I check for the limit in the generation of successors block, using that to decide whether to put them into the stack. I think this is still logically the same though (both in node generation/ before node expansion if i have the terminology right)

For part3, heuristic.
The heuristic i used was MST. I used it because it was the most dominant one described in the tutorials, and testing all the other ones i tried, this was true.
I think it works well because pacman needs to travel between all the foods, and MST gives the smallest value that connects them all. And because pacman doesn't need to touch a certain point, all that matters is the food so MST would give the best path for each position pacman is at(discounting walls because I couldnt get mazeDistance working). I'm  thinking that maybe with more complicated layouts it wouldn't be, but i dont remember enough about graphs to confidently say.
Ended up using manhattan to find the "cost" between two points, for the spanning. To take into account the capsules, I counted the largest number of them in the grid bounded by the two positions, up to a max spaces of the line between the two.

From the solutions, I also tried goal counter, it worked, MST better.
cPH2 also utilizes manhattan distance and counting capsules in a grid area. I think its doing the furthest goal heuristic accounting for capsules in a similar way to how i did? With goal count as a max as well.
is cPH3 doing a BrFS to get the distance between two positions? So its using search instead of manhattan distance to get the furtherst goal. Also with goal count as a max.
