package se.bitcraze.crazyfliecontrol2.tune;

import android.content.Context;
import android.graphics.Color;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * One parameter row: label, numeric field, -10% / +10% buttons and the one-line
 * "too low / too high" hint under it. Writing a value calls back into the activity,
 * which does the {@code Param.setValue}; {@link #onParamValue} shows values the drone
 * reports back so external changes appear.
 */
public class ParamEditor extends LinearLayout {

    public interface OnCommitListener {
        void onCommit(ParamSpec spec, double value);
    }

    private final ParamSpec mSpec;
    private final EditText mValue;
    private boolean mUpdating;   // true while we push a read-back into the field

    public ParamEditor(Context context, ParamSpec spec, final OnCommitListener listener) {
        super(context);
        mSpec = spec;
        setOrientation(VERTICAL);

        TextView label = new TextView(context);
        label.setText(spec.label + "  (" + spec.completeName + ")");
        label.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
        label.setTextColor(0xFF202428);
        label.setGravity(Gravity.START);
        addView(label, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);

        mValue = new EditText(context);
        mValue.setInputType(InputType.TYPE_CLASS_NUMBER
                | InputType.TYPE_NUMBER_FLAG_DECIMAL
                | InputType.TYPE_NUMBER_FLAG_SIGNED);
        mValue.setImeOptions(EditorInfo.IME_ACTION_DONE);
        mValue.setSingleLine(true);
        mValue.setTextSize(TypedValue.COMPLEX_UNIT_SP, 14);
        mValue.setGravity(Gravity.END);
        LayoutParams valueLp = new LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f);
        valueLp.rightMargin = dp(6);
        row.addView(mValue, valueLp);

        row.addView(stepButton(context, "−10%", listener, -1), stepButtonParams());
        row.addView(stepButton(context, "+10%", listener, +1), stepButtonParams());
        addView(row, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        TextView hint = new TextView(context);
        hint.setText(spec.hint);
        hint.setTextSize(TypedValue.COMPLEX_UNIT_SP, 11);
        hint.setTextColor(Color.parseColor("#5A6068"));
        hint.setGravity(Gravity.START);
        addView(hint, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        // Losing focus or hitting Done writes the value; the +/- buttons write directly.
        mValue.setOnFocusChangeListener(new OnFocusChangeListener() {
            @Override
            public void onFocusChange(View v, boolean hasFocus) {
                if (!hasFocus) {
                    commit(listener);
                }
            }
        });
        mValue.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView v, int actionId, android.view.KeyEvent event) {
                if (actionId == EditorInfo.IME_ACTION_DONE) {
                    commit(listener);
                    return true;
                }
                return false;
            }
        });
        mValue.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
            }

            @Override
            public void afterTextChanged(Editable s) {
                // nothing to do; commits are explicit so half-typed values stay local
            }
        });
    }

    private Button stepButton(Context context, String text, final OnCommitListener listener, final int sign) {
        Button b = new Button(context);
        b.setText(text);
        b.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        b.setMinHeight(dp(32));
        b.setPadding(dp(10), 0, dp(10), 0);
        b.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                double v0 = currentValue();
                double delta = Math.abs(v0) * 0.1;
                if (delta == 0) {
                    // 10 % of zero is zero; fall back to a sensible increment
                    delta = mSpec.step;
                }
                double nv = mSpec.clamp(v0 + sign * delta);
                mValue.setText(mSpec.format(nv));
                listener.onCommit(mSpec, nv);
            }
        });
        return b;
    }

    private LayoutParams stepButtonParams() {
        LayoutParams lp = new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT);
        lp.leftMargin = dp(4);
        return lp;
    }

    private void commit(OnCommitListener listener) {
        if (mUpdating) {
            return;
        }
        double v = currentValue();
        double clamped = mSpec.clamp(v);
        if (clamped != v) {
            mValue.setText(mSpec.format(clamped));
        }
        listener.onCommit(mSpec, clamped);
    }

    private double currentValue() {
        try {
            return Double.parseDouble(mValue.getText().toString().trim());
        } catch (NumberFormatException e) {
            return mSpec.min;
        }
    }

    /** Read-back from the drone; ignored while the user is editing the field. */
    public void onParamValue(Number value) {
        if (mValue.hasFocus()) {
            return;
        }
        mUpdating = true;
        mValue.setText(mSpec.format(value.doubleValue()));
        mUpdating = false;
    }

    public ParamSpec getSpec() {
        return mSpec;
    }

    private int dp(int v) {
        return (int) (v * getResources().getDisplayMetrics().density + 0.5f);
    }
}
