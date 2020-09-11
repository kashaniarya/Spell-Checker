# Spell-Checker
Your work on this project begins with a partially-working spell checker. It allows a user to specify some configuration via the standard input (i.e., std::cin) and writes its output to standard output (i.e., std::cout). In short, it is capable of doing two different things:
1. Given a file containing a word set and another file containing input text, it can check spelling in the input text and report on misspellings, along with suggestions about potentially correct spellings.
2. Given a file containing a word set and another file containing input text, it can do the work of a spell checker without displaying a result, instead displaying the CPU time required to perform the task. This allows us to test different search structures to see what effect they have on performance.

Much of the program is already written and its complete source code has been provided. I will only be requiring you to implement two relatively small parts of it:
A class called WordChecker that checks the spelling of words and makes appropriate suggestions when they're misspelled.
Three class templates that derive from an existing class template Set<ElementType>, which implements the concept of a set (i.e., a collection of search keys). Only a handful of operations is supported in each, and you are not required to implement anything not declared in Set<ElementType>, other than a handful of additional functions that is specific to each (which are there to allow us to test details about its internal structure). There are actually three different class templates that you can write, but you are only required to choose two of them — you can choose any two of the three you'd like.
AVLSet<ElementType>, which is an AVL tree (or can be configured to skip the balancing and act as a binary search tree). Substantial partial credit is available on this one if you correctly implement only the binary search tree with no balancing.
SkipListSet<ElementType>, which is a skip list.
HashSet<ElementType>, which is a hash table with separate chaining, implemented as a
dynamically-allocated array of linked lists.
