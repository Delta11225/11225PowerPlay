package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Wrapper for telemetry. Takes FTC telemetry and Dashboard telemetry and passes method calls to
 * both.
 */
@Deprecated
class TelemWrapper implements Telemetry {
    private Telemetry dashboardTelem = null;
    private Telemetry originalTelem = null;

    public TelemWrapper(Telemetry originalTelemetry, Telemetry dashboardTelemetry) {
        dashboardTelem = dashboardTelemetry;
        originalTelem = originalTelemetry;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        dashboardTelem.addData(caption, format, args);
        return originalTelem.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        dashboardTelem.addData(caption, value);
        return originalTelem.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        dashboardTelem.addData(caption, valueProducer);
        return originalTelem.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        dashboardTelem.addData(caption, format, valueProducer);
        return originalTelem.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        dashboardTelem.removeItem(item);
        return originalTelem.removeItem(item);
    }

    @Override
    public void clear() {
        dashboardTelem.clear();
        originalTelem.clear();
    }

    @Override
    public void clearAll() {
        dashboardTelem.clearAll();
        originalTelem.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        dashboardTelem.addAction(action);
        return originalTelem.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        dashboardTelem.removeAction(token);
        return originalTelem.removeAction(token);
    }

    @Override
    public void speak(String text) {
        dashboardTelem.speak(text);
        originalTelem.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        dashboardTelem.speak(text, languageCode, countryCode);
        originalTelem.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        dashboardTelem.update();
        return originalTelem.update();
    }

    @Override
    public Line addLine() {
        dashboardTelem.addLine();
        return originalTelem.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        dashboardTelem.addLine(lineCaption);
        return originalTelem.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        dashboardTelem.removeLine(line);
        return originalTelem.removeLine(line);
    }

    @Override
    public boolean isAutoClear() {
        return originalTelem.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        dashboardTelem.setAutoClear(autoClear);
        originalTelem.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return originalTelem.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        dashboardTelem.setMsTransmissionInterval(msTransmissionInterval);
        originalTelem.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return originalTelem.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        dashboardTelem.setItemSeparator(itemSeparator);
        originalTelem.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return originalTelem.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        dashboardTelem.setCaptionValueSeparator(captionValueSeparator);
        originalTelem.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        dashboardTelem.setDisplayFormat(displayFormat);
        originalTelem.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return originalTelem.log();
    }
}
