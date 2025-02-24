import {createSlice} from "@reduxjs/toolkit";

const initialState = {
  lastNetworkData: {nodes: [], links: []},
}

export const gDataSlice = createSlice({
    name: 'gData',
    initialState,
    reducers: {
        setLastNetworkData: (state, action) => {
            const {payload} = action;
            const {lastNetworkData} = payload;
            state.lastNetworkData = lastNetworkData;
        },
    }
})

export const {
    setLastNetworkData,
} = gDataSlice.actions;

export default gDataSlice.reducer;