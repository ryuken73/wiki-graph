import React from 'react';
import styled from 'styled-components';
import CustomCheckbox from './CustomCheckbox';
import {getBacklinksByContentId} from '../js/serverApi.js';
import {
  expandNetworkData, 
  extractBacklinks,
  extractForwardlinks
} from '../js/dataHandlers.js';
import {
  RowContainer,
  Node,
  Title,
  DelButton,
  TimeStamp
} from './NodeStyles.js'

function NodeExpanded(props) {
  const {
    node, 
    checkedNodeList,
    removeNode, 
    lastNetworkData,
    setLastNetworkData, 
    setBacklinksToShow,
    setForwardlinksToShow,
    setActiveExpandedNodeId,
    addCheckedNodeList,
    delCheckedNodeList,
    focusNode
  } = props;
  const [isShowAll, setIsShowAll] = React.useState(false);
  const onClickTitle = React.useCallback((event) => {
    const id = event.target.id;
    const clickedNode = lastNetworkData.nodes.find(node => node.id === id);
    setActiveExpandedNodeId(id);
    const backlinkNodes = extractBacklinks(lastNetworkData, id);
    setBacklinksToShow(backlinkNodes)
    console.log('backlinkNodes', backlinkNodes)
    const forwardlinkNodes = extractForwardlinks(lastNetworkData, id);
    setForwardlinksToShow(forwardlinkNodes)
    console.log('forwardlinkNodes', forwardlinkNodes)
    focusNode(clickedNode)
  }, [lastNetworkData])
  const toggleShowsAllBacklinks = React.useCallback((event) => {
    const {id} = event.target;
    setIsShowAll( async (isShowAll) => {
      const includeOnlyContents = isShowAll;
      const rows = await getBacklinksByContentId(id)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = expandNetworkData(rows, id, lastNetworkData, includeOnlyContents);
        return newNetworkData
      })
    })
  }, [])
  const setChecked = React.useCallback((checked, checkedId) => {
    console.log(checked, checkedId)
    if(checked){
      addCheckedNodeList(checkedId);
    } else {
      delCheckedNodeList(checkedId);
    }
  }, [])
  const isChecked = checkedNodeList.some(nodeChecked => nodeChecked.id === node.id)
  return (
    <RowContainer>
      <Node key={node.id}>
        <CustomCheckbox
          id={node.id}
          checked={isChecked}
          setChecked={setChecked} 
        ></CustomCheckbox>
        <Title
          id={node.id}
          onClick={onClickTitle}
        >
          {node.text}
        </Title>
        <DelButton id={node.id} onClick={removeNode}>
          [Del]
        </DelButton>
      </Node>
      <TimeStamp
        onClick={toggleShowsAllBacklinks}
        id={node.id}
      >
        backlinks [{node.backlinkCount}]
      </TimeStamp>
    </RowContainer>
  )
}

export default React.memo(NodeExpanded)