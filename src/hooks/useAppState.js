import React from 'react';
import {useSelector, useDispatch} from 'react-redux';
import {
    setShowBackdrop
} from '../appSlice';

export default function useAppState() {
    const dispatch = useDispatch();
    const showBackdrop = useSelector(state => state.app.showBackdrop);

    const setShowBackdropState = React.useCallback(showBackdrop => {
        dispatch(setShowBackdrop({showBackdrop}))
    },[dispatch])

    return {
      showBackdrop,
      setShowBackdropState
    }
}