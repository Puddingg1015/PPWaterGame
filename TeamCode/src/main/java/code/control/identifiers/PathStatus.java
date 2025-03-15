package code.control.identifiers;

public enum PathStatus implements PathNavStatus {
    START, PRELOAD, PARK, END, BUSY,
    ;
    public enum Sample implements PathNavStatus {
        SAMPLE1_PICKUP, SAMPLE2_PICKUP, SAMPLE3_PICKUP, SAMPLE4_PICKUP,
        SAMPLE1_SCORE, SAMPLE2_SCORE, SAMPLE3_SCORE, SAMPLE4_SCORE,
    }
    ;
    public enum Specimen implements PathNavStatus {
        SPEC1_PICKUP, SPEC2_PICKUP, SPEC3_PICKUP, SPEC4_PICKUP,
        SPEC1_SCORE, SPEC2_SCORE, SPEC3_SCORE, SPEC4_SCORE,
    }
    ;
}
