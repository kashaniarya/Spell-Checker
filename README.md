# Spell-Checker
Your work on this project begins with a partially-working spell checker. It allows a user to specify some configuration via the standard input (i.e., std::cin) and writes its output to standard output (i.e., std::cout). In short, it is capable of doing two different things:
1. Given a file containing a word set and another file containing input text, it can check spelling in the input text and report on misspellings, along with suggestions about potentially correct spellings.
2. Given a file containing a word set and another file containing input text, it can do the work of a spell checker without displaying a result, instead displaying the CPU time required to perform the task. This allows us to test different search structures to see what effect they have on performance.
