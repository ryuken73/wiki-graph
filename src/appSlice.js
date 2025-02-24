import {createSlice} from "@reduxjs/toolkit";

const initialState = {
  showBackdrop: false,
}

export const appSlice = createSlice({
    name: 'app',
    initialState,
    reducers: {
        setShowBackdrop: (state, action) => {
            const {payload} = action;
            const {showBackdrop} = payload;
            state.showBackdrop = showBackdrop;
        },
    }
})

export const {
    setShowBackdrop,
} = appSlice.actions;

export default appSlice.reducer;