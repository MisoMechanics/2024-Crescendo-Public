package team9442.frc2024.vision;

import java.util.Set;

public class TagLimiter {

    private final Set<Integer> allowTags;

    public TagLimiter(Set<Integer> allowTags) {
        this.allowTags = allowTags;
    }

    public boolean useTag(int tag) {
        return allowTags.contains(tag);
    }

    public boolean addTag(int tag) {
        return allowTags.add(tag);
    }

    public boolean removeTag(int tag) {
        return allowTags.remove(tag);
    }
}
