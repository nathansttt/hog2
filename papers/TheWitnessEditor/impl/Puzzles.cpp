//
//  PaperPuzzles.cpp
//  The Witness Editor
//
//  Created by Samarium on 2023-08-15.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#include "Puzzles.h"

void Fig6Puzzle1()
{
    witness.AddSeparationConstraint(0, 0, Colors::black);
    witness.AddSeparationConstraint(0, 1, Colors::black);
    witness.AddSeparationConstraint(0, 2, Colors::black);
    witness.AddSeparationConstraint(0, 3, Colors::black);
    
    witness.AddSeparationConstraint(1, 3, Colors::black);
    witness.AddSeparationConstraint(2, 3, Colors::black);
    
    witness.AddSeparationConstraint(3, 0, Colors::black);
    witness.AddSeparationConstraint(3, 1, Colors::black);
    witness.AddSeparationConstraint(3, 2, Colors::black);
    witness.AddSeparationConstraint(3, 3, Colors::black);
    
    witness.AddSeparationConstraint(1, 0, Colors::lightblue);
    witness.AddSeparationConstraint(1, 1, Colors::lightblue);
    witness.AddSeparationConstraint(1, 2, Colors::lightblue);
    
    witness.AddSeparationConstraint(2, 0, Colors::lightblue);
    witness.AddSeparationConstraint(2, 1, Colors::lightblue);
    witness.AddSeparationConstraint(2, 2, Colors::lightblue);
}

void Fig6Puzzle2()
{
    witness.AddSeparationConstraint(0, 0, Colors::lightblue);
    witness.AddSeparationConstraint(1, 0, Colors::black);
    witness.AddSeparationConstraint(0, 1, Colors::black);
    witness.AddSeparationConstraint(1, 1, Colors::black);
    
    witness.AddSeparationConstraint(0, 2, Colors::lightblue);
    witness.AddSeparationConstraint(0, 3, Colors::lightblue);
    witness.AddSeparationConstraint(1, 2, Colors::lightblue);
    witness.AddSeparationConstraint(1, 3, Colors::black);
    
    witness.AddSeparationConstraint(2, 0, Colors::lightblue);
    witness.AddSeparationConstraint(2, 1, Colors::lightblue);
    witness.AddSeparationConstraint(2, 2, Colors::lightblue);
    witness.AddSeparationConstraint(2, 3, Colors::lightblue);
    
    witness.AddSeparationConstraint(3, 0, Colors::black);
    witness.AddSeparationConstraint(3, 1, Colors::black);
    witness.AddSeparationConstraint(3, 2, Colors::black);
    witness.AddSeparationConstraint(3, 3, Colors::black);
    
    witness.AddCannotCrossConstraint(true, 0, 0);
    witness.AddCannotCrossConstraint(true, 2, 0);

    witness.AddCannotCrossConstraint(true, 1, 1);
    witness.AddCannotCrossConstraint(true, 2, 1);
    witness.AddCannotCrossConstraint(true, 3, 1);

    witness.AddCannotCrossConstraint(true, 2, 2);
    witness.AddCannotCrossConstraint(true, 3, 2);

    witness.AddCannotCrossConstraint(true, 0, 3);
    witness.AddCannotCrossConstraint(true, 2, 3);
    witness.AddCannotCrossConstraint(true, 3, 3);

    witness.AddCannotCrossConstraint(true, 1, 4);
    witness.AddCannotCrossConstraint(true, 3, 4);
    
    witness.AddCannotCrossConstraint(false, 0, 1);
    witness.AddCannotCrossConstraint(false, 1, 1);
    
    witness.AddCannotCrossConstraint(false, 1, 2);
    witness.AddCannotCrossConstraint(false, 2, 2);
}

void Fig6Puzzle3()
{
    witness.AddSeparationConstraint(0, 0, Colors::lightblue);
    witness.AddSeparationConstraint(1, 0, Colors::black);
    witness.AddSeparationConstraint(2, 0, Colors::black);
    
    witness.AddSeparationConstraint(2, 1, Colors::lightblue);
    witness.AddSeparationConstraint(3, 1, Colors::black);
    
    witness.AddSeparationConstraint(0, 2, Colors::lightblue);
    witness.AddSeparationConstraint(1, 2, Colors::black);
    
    witness.AddSeparationConstraint(1, 3, Colors::lightblue);
    witness.AddSeparationConstraint(2, 3, Colors::lightblue);
    witness.AddSeparationConstraint(3, 3, Colors::black);
    
    witness.AddMustCrossConstraint(false, 2, 1);
    witness.AddMustCrossConstraint(true, 3, 2);
    witness.AddMustCrossConstraint(true, 0, 4);
}

void Fig6Puzzle4()
{
    witness.AddSeparationConstraint(0, 0, Colors::black);
    witness.AddSeparationConstraint(1, 0, Colors::black);
    witness.AddSeparationConstraint(3, 0, Colors::lightblue);
    
    witness.AddSeparationConstraint(1, 1, Colors::black);
    witness.AddSeparationConstraint(2, 1, Colors::black);
    
    witness.AddSeparationConstraint(0, 2, Colors::lightblue);
    witness.AddSeparationConstraint(1, 2, Colors::black);
    witness.AddSeparationConstraint(3, 2, Colors::black);
    
    witness.AddSeparationConstraint(1, 3, Colors::lightblue);
    witness.AddSeparationConstraint(2, 3, Colors::black);
    witness.AddSeparationConstraint(3, 3, Colors::lightblue);
}

void k87fxsr()
{
    witness.AddSeparationConstraint(0, 0, Colors::cyan);
    witness.AddSeparationConstraint(0, 3, Colors::cyan);
    
    witness.AddSeparationConstraint(1, 1, Colors::yellow);
    witness.AddSeparationConstraint(2, 1, Colors::yellow);
    
    witness.AddSeparationConstraint(1, 2, Colors::lightgray);
    witness.AddSeparationConstraint(2, 2, Colors::lightgray);
    
    witness.AddSeparationConstraint(3, 0, Colors::magenta);
    witness.AddSeparationConstraint(3, 3, Colors::magenta);
//    witness.AddTriangleConstraint(3, 3, 1);
//    witness.AddTriangleConstraint(1, 0, 2);
//    witness.AddTriangleConstraint(2, 0, 3);

}
