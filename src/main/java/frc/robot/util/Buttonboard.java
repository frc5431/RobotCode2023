package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Buttonboard extends CommandGenericHID {
    int width;
    int height;
    static HashMap<Pair<Alphabet, Integer>, Trigger> cachedButtons = new HashMap<>();

    public Buttonboard(int port, int width, int height) {
        super(port);
        this.width = width;
        this.height = height;
    }

    public Trigger buttonAt(Alphabet vertical, int horizontal) {
        var pair = new Pair<>(vertical, horizontal);
        if (cachedButtons.containsKey(pair)) {
            return cachedButtons.get(pair);
        }

        // This starts it off as 0
        int y = vertical.getIndex();
        int x = horizontal;
        if (x <= 0) {
            throw new IndexOutOfBoundsException("Number starts at 1, not 0");
        }
        if(x > width) {
            throw new IndexOutOfBoundsException("Number, " + x + " is larger than your button board width, " + width);
        }
        if(y > height) {
            throw new IndexOutOfBoundsException("Letter at index " + y + " is larger than your button board hieght, " + height);
        }

        Trigger foundTrigger = button(y * width + x);
        cachedButtons.put(pair, foundTrigger);

        return foundTrigger;
    }

    public ButtonBoardIterationBuilder iterate() {
        return new ButtonBoardIterationBuilder(this);
    }

    public Trigger A1() {
        return buttonAt(Alphabet.A, 1);
    }

    public Trigger A2() {
        return buttonAt(Alphabet.A, 2);
    }

    public Trigger A3() {
        return buttonAt(Alphabet.A, 3);
    }

    public Trigger A4() {
        return buttonAt(Alphabet.A, 4);
    }

    public Trigger A5() {
        return buttonAt(Alphabet.A, 5);
    }

    public Trigger A6() {
        return buttonAt(Alphabet.A, 6);
    }

    public Trigger A7() {
        return buttonAt(Alphabet.A, 7);
    }

    public Trigger B1() {
        return buttonAt(Alphabet.B, 1);
    }

    public Trigger B2() {
        return buttonAt(Alphabet.B, 2);
    }

    public Trigger B3() {
        return buttonAt(Alphabet.B, 3);
    }

    public Trigger B4() {
        return buttonAt(Alphabet.B, 4);
    }

    public Trigger B5() {
        return buttonAt(Alphabet.B, 5);
    }

    public Trigger B6() {
        return buttonAt(Alphabet.B, 6);
    }

    public Trigger B7() {
        return buttonAt(Alphabet.B, 7);
    }

    public Trigger C1() {
        return buttonAt(Alphabet.C, 1);
    }

    public Trigger C2() {
        return buttonAt(Alphabet.C, 2);
    }

    public Trigger C3() {
        return buttonAt(Alphabet.C, 3);
    }

    public Trigger C4() {
        return buttonAt(Alphabet.C, 4);
    }

    public Trigger C5() {
        return buttonAt(Alphabet.C, 5);
    }

    public Trigger C6() {
        return buttonAt(Alphabet.C, 6);
    }

    public Trigger C7() {
        return buttonAt(Alphabet.C, 7);
    }

    public enum Alphabet {
        A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z;

        private Alphabet() { }

        public int getIndex() {
            return this.ordinal()+1;
        }

        public String getLetter() {
            return this.name();
        }

        public static Alphabet fromIndex(int index) {
            try {
                return Alphabet.values()[index-1];
            } catch (ArrayIndexOutOfBoundsException e) {
                throw new IllegalArgumentException("Invalid index: " + index);
            }
        }
    }
}
