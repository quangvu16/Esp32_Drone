package com.MobileAnarchy.Android.Widgets.Joystick;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;

import se.bitcraze.crazyfliecontrol.prefs.PreferencesActivity;

public class JoystickView extends View {
    public static final int INVALID_POINTER_ID = -1;
    private static final String TAG = "JoystickView";

    private boolean isLeft = true;

    private Paint bgPaint;
    private Paint handlePaint;

    private int innerPadding;
    private int bgRadius;
    private int handleRadius;
    private int movementRadius;
    private int handleInnerBoundaries;

    private JoystickMovedListener moveListener;

    // # of pixels movement required between reporting to the listener
    // Số pixel cần di chuyển giữa hai lần báo cho listener
    private float moveResolution;

    public final static int AUTO_RETURN_NONE = 0;
    public final static int AUTO_RETURN_CENTER = 1;
    public final static int AUTO_RETURN_BOTTOM = 2;
    private int autoReturnMode;
    private volatile int autoReturnSequenceNum;

    // Max range of movement in user coordinate system
    // Biên độ di chuyển tối đa trong hệ tọa độ người dùng
    public final static int CONSTRAIN_BOX = 0;
    public final static int CONSTRAIN_CIRCLE = 1;
    private int movementConstraint;
    private float movementRange;

    public final static int COORDINATE_CARTESIAN = 0; // Regular cartesian coordinates
    // Tọa độ Descartes thông thường
    public final static int COORDINATE_DIFFERENTIAL = 1; // Uses polar rotation of 45 degrees to calc differential drive parameters
    // Dùng phép xoay cực 45 độ để tính tham số dẫn động sai phân
    private int userCoordinateSystem;

    private float prefRatio = 1;

    // Last touch point in view coordinates
    // Điểm chạm cuối cùng trong tọa độ của view
    private int pointerId = INVALID_POINTER_ID;
    private float touchX, touchY;

    // Last reported position in view coordinates (allows different reporting sensitivities)
    // Vị trí được báo cuối cùng trong tọa độ của view (cho phép độ nhạy báo cáo khác nhau)
    private float reportX, reportY;

    // Handle center in view coordinates
    // Tâm của tay cầm trong tọa độ của view
    private float handleX, handleY;

    // Center of the view in view coordinates
    // Tâm của view trong tọa độ của view
    private int circleCenterX, circleCenterY;

    // Size of the view in view coordinates
    // Kích thước của view trong tọa độ của view
    private int dimX, dimY;

    // Cartesian coordinates of last touch point - joystick center is (0,0)
    // Tọa độ Descartes của điểm chạm cuối cùng - tâm joystick là (0,0)
    private int cartX, cartY;

    // Polar coordinates of the touch point from joystick center
    // Tọa độ cực của điểm chạm tính từ tâm joystick
    private double radial;
    private double angle;

    // User coordinates of last touch point
    // Tọa độ người dùng của điểm chạm cuối cùng
    private float userX, userY;

    // Offset co-ordinates (used when touch events are received from parent's coordinate origin)
    // Tọa độ độ lệch (dùng khi sự kiện chạm nhận từ gốc tọa độ của view cha)
    private int offsetX;
    private int offsetY;

    private boolean verticalLocked = false;
    private boolean horizontalLocked = false;

    // =========================================
    // Constructors
    // =========================================

    public JoystickView(Context context) {
        super(context);
        initJoystickView();
    }

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initJoystickView();
    }

    public JoystickView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        initJoystickView();
    }

    // =========================================
    // Initialization
    // =========================================

    private void initJoystickView() {
        setFocusable(true);

        bgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        bgPaint.setColor(Color.BLACK);
        bgPaint.setStrokeWidth(2);
        bgPaint.setStyle(Paint.Style.STROKE);

        handlePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        handlePaint.setColor(Color.GRAY);
        handlePaint.setStrokeWidth(1);
        handlePaint.setAlpha(100);
        handlePaint.setStyle(Paint.Style.FILL_AND_STROKE);

        innerPadding = 10;

        setMovementRange(10);
        setMoveResolution(1.0f);
        setUserCoordinateSystem(COORDINATE_CARTESIAN);
        setAutoReturnToCenter(true);
    }

    public void setAutoReturnToCenter(boolean autoReturnToCenter) {
        this.autoReturnMode = autoReturnToCenter ? AUTO_RETURN_CENTER : AUTO_RETURN_NONE;
    }

    public void setAutoReturnMode(int autoReturnMode) {
        this.autoReturnMode = autoReturnMode;
    }

    private void setUserCoordinateSystem(int userCoordinateSystem) {
        if (userCoordinateSystem < COORDINATE_CARTESIAN || movementConstraint > COORDINATE_DIFFERENTIAL) {
            Log.e(TAG, "invalid value for userCoordinateSystem");
        } else {
            this.userCoordinateSystem = userCoordinateSystem;
        }
    }

    public void setMovementConstraint(int movementConstraint) {
        if (movementConstraint < CONSTRAIN_BOX || movementConstraint > CONSTRAIN_CIRCLE) {
            Log.e(TAG, "invalid value for movementConstraint");
        } else {
            this.movementConstraint = movementConstraint;
        }
    }

    public void setMovementRange(float movementRange) {
        this.movementRange = movementRange;
    }

    private void setMoveResolution(float moveResolution) {
        this.moveResolution = moveResolution;
    }

    public void setOnJoystickMovedListener(JoystickMovedListener listener) {
        this.moveListener = listener;
    }

    // =========================================
    // Drawing Functionality
    // =========================================

    // onSizeChanged?

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);
        // Here we make sure that we have a perfect circle
        // Ở đây ta đảm bảo có một hình tròn hoàn hảo
        int measuredWidth = measure(widthMeasureSpec);
        int measuredHeight = measure(heightMeasureSpec);
        setMeasuredDimension(measuredWidth, measuredHeight);
    }

    @Override
    protected void onLayout(boolean changed, int left, int top, int right, int bottom) {
        super.onLayout(changed, left, top, right, bottom);

        int width = getMeasuredWidth();
        int height = getMeasuredHeight();
        int size = Math.min(width, height);

        size = Math.round(size*prefRatio);

        dimX = size;
        dimY = size;

        if (!isLeft && (width > size)){
            //add x offset to right joystick when size is smaller than width
            // thêm độ lệch x cho joystick bên phải khi kích thước nhỏ hơn chiều rộng
            circleCenterX = (width-size) + (size / 2);
            setTouchOffsetX(width-size);
        } else {
            circleCenterX = size / 2;
            setTouchOffsetX(0);
        }
        if (height > size) {
            //add y offset to joystick when size is smaller than height
            // thêm độ lệch y cho joystick khi kích thước nhỏ hơn chiều cao
            circleCenterY = (height-size) + (size / 2);
            setTouchOffsetY(height-size);
        } else {
            circleCenterY = size / 2;
            setTouchOffsetY(0);
        }

        bgRadius = dimX / 2 - innerPadding;
        handleRadius = (int) (size * 0.20);
        handleInnerBoundaries = handleRadius;
        float oldMovementRadius = movementRadius;
        movementRadius = (size / 2) - handleInnerBoundaries;
        if(oldMovementRadius != movementRadius) {
            autoReturn(true);
        }
    }

    private int measure(int measureSpec) {
        int result = 0;
        // Decode the measurement specifications.
        // Giải mã các thông số đo.
        int specMode = MeasureSpec.getMode(measureSpec);
        int specSize = MeasureSpec.getSize(measureSpec);
        if (specMode == MeasureSpec.UNSPECIFIED) {
            // Return a default size of 200 if no bounds are specified.
            // Trả về kích thước mặc định 200 nếu không chỉ định giới hạn.
            result = 200;
        } else {
            // As you want to fill the available space always return the full available bounds.
            // Vì muốn lấp đầy không gian có sẵn nên luôn trả về toàn bộ giới hạn khả dụng.
            result = specSize;
        }
        return result;
    }

    public void setPreferences(SharedPreferences prefs){
        prefRatio = Float.parseFloat(prefs.getString(PreferencesActivity.KEY_PREF_JOYSTICK_SIZE, "100"));
        prefRatio/=100.0;
        requestLayout();
    }

    @Override
    protected void onDraw(Canvas canvas) {
//        canvas.save();
        // Draw the background
        // Vẽ nền
        canvas.drawCircle(circleCenterX, circleCenterY, bgRadius, bgPaint);

        // Draw the handle
        // Vẽ tay cầm
        handleX = touchX + circleCenterX;
        handleY = touchY + circleCenterY;
        canvas.drawCircle(handleX, handleY, handleRadius, handlePaint);
        // Log.d(TAG, String.format("touch(%f,%f)", touchX, touchY));
        // Log.d(TAG, String.format("onDraw(%.1f,%.1f)\n\n", handleX, handleY));
//        canvas.restore();
    }

    // Constrain touch within a box
    // Giới hạn điểm chạm trong một hình vuông
    private void constrainBox() {
        touchX = Math.max(Math.min(touchX, movementRadius), -movementRadius);
        touchY = Math.max(Math.min(touchY, movementRadius), -movementRadius);
    }

    // Constrain touch within a circle
    // Giới hạn điểm chạm trong một hình tròn
    private void constrainCircle() {
        float diffX = touchX;
        float diffY = touchY;
        double radial = Math.sqrt((diffX * diffX) + (diffY * diffY));
        if (radial > movementRadius) {
            touchX = (int) ((diffX / radial) * movementRadius);
            touchY = (int) ((diffY / radial) * movementRadius);
        }
    }

    private void setPointerId(int id) {
        this.pointerId = id;
    }

    @Override
    public boolean onTouchEvent(MotionEvent ev) {
        final int action = ev.getAction();
        switch (action & MotionEvent.ACTION_MASK) {
        case MotionEvent.ACTION_MOVE: {
            return processMoveEvent(ev);
        }
        case MotionEvent.ACTION_CANCEL:
        case MotionEvent.ACTION_UP: {
            if (pointerId != INVALID_POINTER_ID) {
                // Log.d(TAG, "ACTION_UP");
                autoReturn(false);
                setPointerId(INVALID_POINTER_ID);
            }
            break;
        }
        case MotionEvent.ACTION_POINTER_UP: {
            if (pointerId != INVALID_POINTER_ID) {
                final int pointerIndex = (action & MotionEvent.ACTION_POINTER_INDEX_MASK) >> MotionEvent.ACTION_POINTER_INDEX_SHIFT;
                final int pointerId = ev.getPointerId(pointerIndex);
                if (pointerId == this.pointerId) {
                    // Log.d(TAG, "ACTION_POINTER_UP: " + pointerId);
                    autoReturn(false);
                    setPointerId(INVALID_POINTER_ID);
                    return true;
                }
            }
            break;
        }
        case MotionEvent.ACTION_DOWN: {
            if (pointerId == INVALID_POINTER_ID) {
                int x = (int) ev.getX();
                int y = (int) ev.getY();
                if (x >= offsetX && x < offsetX + dimX && y >= offsetY && y < offsetY + dimY) {
                    setPointerId(ev.getPointerId(0));
                    // Log.d(TAG, "ACTION_DOWN: " + getPointerId());
                    return true;
                }
            }
            break;
        }
        case MotionEvent.ACTION_POINTER_DOWN: {
            if (pointerId == INVALID_POINTER_ID) {
                final int pointerIndex = (action & MotionEvent.ACTION_POINTER_INDEX_MASK) >> MotionEvent.ACTION_POINTER_INDEX_SHIFT;
                final int pointerId = ev.getPointerId(pointerIndex);
                int x = (int) ev.getX(pointerIndex);
                int y = (int) ev.getY(pointerIndex);
                if (x >= offsetX && x < offsetX + dimX && y >= offsetY && y < offsetY + dimY) {
                    // Log.d(TAG, "ACTION_POINTER_DOWN: " + pointerId);
                    setPointerId(pointerId);
                    return true;
                }
            }
            break;
        }
        }
        return false;
    }

    private boolean processMoveEvent(MotionEvent ev) {
        if (pointerId != INVALID_POINTER_ID) {
            final int pointerIndex = ev.findPointerIndex(pointerId);

            // Translate touch position to center of view
            // Chuyển vị trí chạm về tâm của view
            float x = ev.getX(pointerIndex);
            touchX = horizontalLocked ? 0 : x - circleCenterX;
            float y = ev.getY(pointerIndex);
            touchY = verticalLocked ? 0 : y - circleCenterY;

            // Log.d(TAG, String.format("ACTION_MOVE: (%03.0f, %03.0f) => (%03.0f, %03.0f)", x, y, touchX, touchY));
            reportOnMoved();
            invalidate();
            return true;
        }
        return false;
    }

    private void reportOnMoved() {
        if (movementConstraint == CONSTRAIN_CIRCLE) {
            constrainCircle();
        } else {
            constrainBox();
        }
        calcUserCoordinates();
        if (moveListener != null) {
            boolean rx = Math.abs(touchX - reportX) >= moveResolution;
            boolean ry = Math.abs(touchY - reportY) >= moveResolution;
            if (rx || ry) {
                this.reportX = touchX;
                this.reportY = touchY;

                // Log.d(TAG, String.format("moveListener.OnMoved(%d,%d)", (int)userX, (int)userY));
                moveListener.OnMoved(userX, userY);
            }
        }
    }

    private void calcUserCoordinates() {
        // First convert to cartesian coordinates
        // Đầu tiên chuyển sang tọa độ Descartes
        cartX = (int) (touchX / movementRadius * movementRange);
        cartY = (int) (touchY / movementRadius * movementRange);

        radial = Math.sqrt((cartX * cartX) + (cartY * cartY));
        angle = Math.atan2(cartY, cartX);

        // Invert Y axis by default
        // Đảo trục Y theo mặc định
        cartY *= -1;

        if (userCoordinateSystem == COORDINATE_CARTESIAN) {
            userX = cartX / movementRange;
            userY = cartY / movementRange;
        } else if (userCoordinateSystem == COORDINATE_DIFFERENTIAL) {
            userX = cartY + cartX / 4;
            userY = cartY - cartX / 4;

            if (userX < -movementRange) {
                userX = (int) -movementRange;
            }
            if (userX > movementRange) {
                userX = (int) movementRange;
            }
            if (userY < -movementRange) {
                userY = (int) -movementRange;
            }
            if (userY > movementRange) {
                userY = (int) movementRange;
            }
        }

    }

    public void autoReturn(boolean immediate) {
        if (autoReturnMode != AUTO_RETURN_NONE) {
            final int numberOfFrames = immediate ? 1 : 5;
            final double intervalsX = (0 - touchX) / numberOfFrames;
            final double returnY = autoReturnMode == AUTO_RETURN_BOTTOM ? movementRadius : 0;
            final double intervalsY = (returnY - touchY) / numberOfFrames;

            ++autoReturnSequenceNum;
            final int thisAutoReturnSequence = autoReturnSequenceNum;

            for (int i = 0; i < numberOfFrames; i++) {
                final int j = i;
                postDelayed(new Runnable() {
                    public void run() {
                        if(thisAutoReturnSequence != autoReturnSequenceNum) {
                            return;
                        }

                        touchX += intervalsX;
                        touchY += intervalsY;

                        reportOnMoved();
                        invalidate();

                        if (moveListener != null && j == numberOfFrames - 1) {
                            moveListener.OnReturnedToCenter();
                        }
                    }
                }, i * 40);
            }

            if (moveListener != null) {
                moveListener.OnReleased();
            }
        }
    }

    private void setTouchOffsetX(int x) {
        offsetX = x;
    }

    private void setTouchOffsetY(int y) {
        offsetY = y;
    }

    public boolean isLeft() {
        return isLeft;
    }

    public void setLeft(boolean left) {
        isLeft = left;
    }

    public void setVerticalLocked(boolean locked) {
        this.verticalLocked = locked;
    }

    public boolean isVerticalLocked() {
        return verticalLocked;
    }

    public void setHorizontalLocked(boolean locked) {
        this.horizontalLocked = locked;
    }

    public boolean isHorizontalLocked() {
        return horizontalLocked;
    }
}
