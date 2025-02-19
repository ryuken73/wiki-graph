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
  LinkCountContainer,
  LinkCount
} from './NodeStyles.js'
import {
  hasInDirectionLink,
  hasOutDirectionLink
} from '../js/graphHandlers.js';

function NodeExpanded(props) {
  const {
    node, 
    checkedNodeList,
    removeNode, 
    expandNode,
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
  const onClickIn = React.useCallback((event) => {
    const isForwarding = false;
    expandNode(node, isForwarding)
  }, [])
  const onClickOut = React.useCallback((event) => {
    const isForwarding = true;
    expandNode(node, isForwarding)
  }, [])
  const setChecked = React.useCallback((checked, checkedId) => {
    console.log(checked, checkedId)
    if(checked){
      addCheckedNodeList(checkedId);
    } else {
      delCheckedNodeList(checkedId);
    }
  }, [])
  const isChecked = checkedNodeList.some(nodeChecked => nodeChecked.id === node.id);
  // const isForwardlinkExpaned = React.useMemo(() => hasOutDirectionLink(node, lastNetworkData.links), lastNetworkData);
  // const isBacklinkExpanded = React.useMemo(() => hasInDirectionLink(node, lastNetworkData.links), lastNetworkData);
  const isForwardlinkExpaned = hasOutDirectionLink(node, lastNetworkData.links);
  const isBacklinkExpanded = hasInDirectionLink(node, lastNetworkData.links);
  console.log(node.text, isForwardlinkExpaned, lastNetworkData)
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
        <LinkCountContainer>
          <LinkCount
            onClick={onClickIn}
            id={node.id}
            disabled={isBacklinkExpanded || node.backlinkCountContent === 0}
          >
            IN [{node.backlinkCountContent}]
          </LinkCount>
          <LinkCount
            onClick={onClickOut}
            id={node.id}
            style={{marginLeft: 'auto'}}
            disabled={isForwardlinkExpaned || node.forwardlinkCount === 0}
          >
            OUT [{node.forwardlinkCount}]
          </LinkCount>
        </LinkCountContainer>
    </RowContainer>
  )
}

export default React.memo(NodeExpanded)