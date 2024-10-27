package team9442.lib;

import java.util.function.*;

public final class FunctionalUtils {

    public static <T> Supplier<T> map(Function<T, T> mapFunc, Supplier<T> supplier) {
        return () -> mapFunc.apply(supplier.get());
    }

    public static DoubleSupplier map(Function<Double, Double> mapFunc, DoubleSupplier supplier) {
        return () -> mapFunc.apply(supplier.getAsDouble());
    }

    private FunctionalUtils() {}
}
