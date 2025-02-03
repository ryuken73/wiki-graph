import React from 'react';
import Box from '@mui/material/Box';
import SnackBar from './Common/SnackBar'
import styled from 'styled-components';
import TextBox from './Common/TextBox';
import HoverButton from './Common/ButtonHover';
import ImageIcon from './Common/ImageIcon';
import colors from '../config/colors';
import ZoomInMapIcon from '@mui/icons-material/ZoomInMap';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';

import expandSvg from '../assets/images/expand.svg'
import shrinkSvg from '../assets/images/shrink.svg'

const ButtonContainer = styled(Box)`
    display: flex;
    flex-direction: row;
    align-items: center;
    justify-content: center;
    flex: 1;
    z-index: 10;
`
const Helper = (props) => {
  const {checkedNodeList=[], clearChecked=()=>{}} = props;
    const checkedCount = checkedNodeList.length;
    const hidden = checkedCount === 0 || checkedCount === false;
    const text = `선택한 ${checkedCount} 노드를`;

    const handleAddCurrentPlaylist = React.useCallback(() => {
        clearChecked();
    },[clearChecked])

    const handleAddCurrentPlaylistNPlay = React.useCallback(() => {
        clearChecked();
    },[clearChecked])

    return (
        <SnackBar hidden={hidden} containerProps={{width:'300px', height:'40px', opacity:'0.9', bgcolor:colors.dark1}}>
            <Box flex="1" justifyContent="center">
                <TextBox fontSize="15px" textAlign="center" color="white" text={text}></TextBox>
            </Box>
            <ButtonContainer>
              <HoverButton onClick={()=>{}}><ZoomInMapIcon fontSize="medium"></ZoomInMapIcon></HoverButton>
              <HoverButton onClick={()=>{}}><ZoomOutMapIcon fontSize="medium"></ZoomOutMapIcon></HoverButton>
            </ButtonContainer>     
        </SnackBar>
    )
}

export default React.memo(Helper)