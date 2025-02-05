import React from 'react';
import Box from '@mui/material/Box';
import SnackBar from './Common/SnackBar'
import styled from 'styled-components';
import TextBox from './Common/TextBox';
import Tooltip from '@mui/material/Tooltip';
import HoverButton from './Common/ButtonHover';
import ImageIcon from './Common/ImageIcon';
import colors from '../config/colors';
import ZoomInMapIcon from '@mui/icons-material/ZoomInMap';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import DeleteIcon from '@mui/icons-material/Delete';

const ButtonContainer = styled(Box)`
    display: flex;
    flex-direction: row;
    align-items: center;
    justify-content: center;
    flex: 1;
`;

const Helper = (props) => {
  const {
    checkedNodeList=[], 
    setCheckedNodeList,
    removeNode
  } = props;
    const checkedCount = checkedNodeList.length;
    const hidden = checkedCount === 0 || checkedCount === false;
    const text = `선택한 ${checkedCount} 노드를`;

    const shrinkCheckedNode = React.useCallback(() => {
    },[])

    const expandCheckedNode = React.useCallback(() => {
    },[])

    const removeCheckedNode = React.useCallback(() => {
      console.log('click remove')
      checkedNodeList.forEach(node => {
        console.log('remove node:', node)
        removeNode(node);
      })
      setCheckedNodeList([]);
    },[checkedNodeList])

    return (
        <SnackBar hidden={hidden} containerProps={{width:'300px', height:'40px', opacity:'0.9', bgcolor:colors.playerLight4}}>
            <Box flex="1" justifyContent="center">
                <TextBox fontSize="15px" textAlign="center" color="white" text={text}></TextBox>
            </Box>
            <ButtonContainer>
              <Tooltip title="그래프 축소">
                <HoverButton onClick={shrinkCheckedNode}><ZoomInMapIcon fontSize="medium"></ZoomInMapIcon></HoverButton>
              </Tooltip>
              <Tooltip title="그래프 확대">
                <HoverButton onClick={expandCheckedNode}><ZoomOutMapIcon fontSize="medium"></ZoomOutMapIcon></HoverButton>
              </Tooltip>
              <Tooltip title="노드 삭제">
                <HoverButton onClick={removeCheckedNode}><DeleteIcon fontSize="medium"></DeleteIcon></HoverButton>
              </Tooltip>
            </ButtonContainer>     
        </SnackBar>
    )
}

export default React.memo(Helper)